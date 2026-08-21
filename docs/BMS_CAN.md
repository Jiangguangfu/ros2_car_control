# BMS CAN 通信说明

BMS（STM32U385）通过 **FDCAN1** 向底盘主控上报电池数据。本板 **无对外 UART**，能源数据 **仅经 CAN** 发送。

协议与 PawDrive-Base-Controller 路线 1 对齐：复用 UART 消息 TYPE 与 payload 布局，CAN 仅作分片物理链路。

## 物理层

| 项目 | 值 |
|------|-----|
| MCU | STM32U385CG |
| 收发器 | SN65HVD230（典型） |
| 引脚 | PA11 = CAN_RX，PA12 = CAN_TX |
| 速率 | **500 kbit/s** |
| 帧类型 | 标准帧 11-bit ID，经典 CAN 8 字节 Data |
| 底盘对接 | 407 CAN2，500 kbit/s（PB12/PB13） |

## 设计原则

| 项目 | 说明 |
|------|------|
| CAN ID | `0x400 + UART TYPE`（见 `can_uart_transport.h`） |
| 分片 | 每帧 Data[8]：`[frag_idx][frag_total][6B payload…]` |
| 字节序 | 小端（LE） |
| 电流符号 | **协议层**：`current_a` 放电为正；**BQ/CC2/CC3**：充电为正、放电为负 |

组包完成后，payload 等价于 UART 收到一条对应 TYPE 的消息。

## 消息一览

| UART TYPE | CAN ID | Payload | 分片数 | 周期 | 说明 |
|-----------|--------|---------|--------|------|------|
| `0x8B` | **0x48B** | 20 B | **4** | **5 Hz**（200 ms） | 电池状态（主包） |
| `0x9A` | **0x49A** | 32 B | **6** | **1 Hz** + 告警变化即发 | 告警 + 扩展测量 |
| `0x9B` | **0x49B** | 8 B | **2** | **1 Hz** + state/mask 变化即发 | 均衡状态监控 |
| `0x41` | **0x441** | 3 B | **1** | **407 下发** | 充电控制（设流 / 启停） |
| `0xA0` | **0x4A0** | 8 B | 1 | 事件 | **充电命令**（底板 → BMS，带仲裁应答） |
| `0xA1` | **0x4A1** | 8 B | 1 | 应答 | **充电仲裁结果** |
| — | 0x48C–0x48F | 8 B × 4 | 1 帧/ID | 5 Hz | **仅联调**（`BMS_CAN_DEBUG=ON`） |

> **注意**：`0x48C`–`0x48F` 为调试帧，与 TYPE `0x8C` 路由规则无关；**量产固件默认关闭**。

---

## 分片格式

```
Byte 0   frag_idx     从 0 递增
Byte 1   frag_total   该 TYPE 固定分片总数
Byte 2–7 payload 连续 6 字节
```

示例 **0x48B** 第 0 帧：`00 04 [6 字节 payload 起始…]`

重组：按 `frag_idx` 顺序拼接，得到完整 payload 后解析结构体。

---

## 0x48B — REPORT_BATTERY_STATE（TYPE 0x8B）

**用途**：主控能源管理主包（SOC、总压、电流、温度）。

**结构体**：`uart_battery_state_report_t`（`uart_battery_report.h`）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | `series_cells` | uint8 | 串联节数（6S = 6） |
| 1 | `present` | uint8 | 1 = 电池存在 |
| 2 | `reserved0` | uint8 | SOH 0~100（容量衰减：实测满充 / 标称；未学习前默认 **100**） |
| 3 | `reserved1` | uint8 | bit0 = `BMS_BATTERY_REPORT_VALID_BIT`（BMS CAN 数据有效） |
| 4–7 | `voltage_v` | float | 包电压（V），来自 BQ Stack `pack_mv` / 回退逻辑 |
| 8–11 | `current_a` | float | 电流（A），**放电为正** |
| 12–15 | `percentage` | float | SOC **0.0~1.0**；未知时为 **-1.0** |
| 16–19 | `temperature_c` | float | 温度（℃）；未知时为 **-1.0** |

**数据来源**：`BmsDataSnapshot_Fill()` ← BQ76942 测量 + `soc_estimator`（SOC/SOH）+ 热管理。

**SOH（方案 B）**：充电会话中库仑计积分；自低 SOC（≤25%）起充至满充，或单次充电增量 ≥ 标称 50% 时，用「起始剩余 + 充入容量」更新 `learned_full`；`SOH = learned_full / 标称 × 100`。断电不保存 learned（重启后 SOH 回到 100% 直至再次满充学习）。

**发送**：`BMS_CanTx_Process()`（`CommTask`，`BMS_CAN_BATTERY_PERIOD_MS` = 200 ms）。

---

## 0x49A — REPORT_BATTERY_EXT（TYPE 0x9A）

**用途**：告警状态 + 明细测量（单体电压、PACK 脚、双通道电流、双温度）。

**结构体**：`uart_battery_ext_report_t`（`uart_battery_ext_report.h`）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0–3 | `alarm_flags` | uint32 | 告警位掩码（见下表） |
| 4 | `severity` | uint8 | 0=无，1=警告，2=严重 |
| 5 | `source_flags` | uint8 | 汇总来源模块位 |
| 6–7 | `output_mv` | uint16 | BQ PACK 脚电压（0x36），mV |
| 8–9 | `vcell_min_mv` | uint16 | 最低单体 mV |
| 10–11 | `vcell_max_mv` | uint16 | 最高单体 mV |
| 12–13 | `current_cc2_ma` | int16 | CC2 电流 mA（BQ 符号：充电 +） |
| 14–15 | `current_cc3_ma` | int16 | CC3 电流 mA |
| 16–17 | `ts1_c_x10` | int16 | TS1 温度 ×10（℃）；无效 **-4000** |
| 18–19 | `ts2_c_x10` | int16 | TS2 温度 ×10；无效 **-4000** |
| 20–31 | `cell_mv[6]` | uint16×6 | 6 节单体电压 mV |

### `source_flags`

| 位 | 宏 | 含义 |
|----|-----|------|
| 0 | `BMS_EXT_SOURCE_BQ_MEAS` | BQ 测量参与 |
| 1 | `BMS_EXT_SOURCE_THERMAL` | 热管理参与 |
| 2 | `BMS_EXT_SOURCE_CHARGE` | 充电管理参与 |
| 3 | `BMS_EXT_SOURCE_BALANCE` | 均衡管理参与 |

### `alarm_flags`

| 位 | 宏 | 含义 |
|----|-----|------|
| 0 | `BMS_EXT_ALARM_OVP` | 过压（单体 ≥ 4200 mV 等） |
| 1 | `BMS_EXT_ALARM_UVP` | 欠压（单体 ≤ 3000 mV 等） |
| 2 | `BMS_EXT_ALARM_OCP` | 过流（充电故障） |
| 3 | `BMS_EXT_ALARM_OVERTEMP` | 过温限流/关断 |
| 4 | `BMS_EXT_ALARM_COLD_CHARGE` | 低温充电限制 |
| 5 | `BMS_EXT_ALARM_BQ_PROTECT` | BQ Safety 保护 |
| 6 | `BMS_EXT_ALARM_IMBALANCE_CHG` | 压差停充 |
| 7 | `BMS_EXT_ALARM_COMM_FAIL` | I2C/通信失败 |
| 8 | `BMS_EXT_ALARM_CHG_INHIBIT` | 充电禁止 |
| 9 | `BMS_EXT_ALARM_DSG_INHIBIT` | 放电禁止 |
| 10 | `BMS_EXT_ALARM_CHARGE_FAULT` | 充电状态机故障 |
| 13 | `BMS_EXT_ALARM_BALANCING` | 正在泄放（`ACTIVE` 或 `MID_PROTECT`），不抬升 severity |
| 14 | `BMS_EXT_ALARM_DELTA_HIGH` | 顶部窗口且 Δ > 15 mV（中段不置位） |

**severity**：有关键告警（BQ 保护、充电故障、过温）时为 **CRITICAL(2)**，其余为 **WARN(1)**。`BALANCING` 单独置位不改变 severity。

明细（阶段、掩码、压差）见 **0x49B**，本帧不扩长度。

**数据来源**：`BmsExtSnapshot_Fill()` ← BQ + 热管理 + 充电 + 均衡 + BQ Safety。

**发送**：`BMS_CanExtTx_Process()`；周期 **1000 ms**，`alarm_flags` 变化时 **立即重发**。

---

## 0x49B — REPORT_BATTERY_BALANCE（TYPE 0x9B）

**用途**：均衡状态监控。不把内部 `balance_status_t` 整包上总线。

压差 `delta_mv` 始终是 `vmax − vmin`，**不分阶段**；阶段用 `state` + `flags.TOP_WINDOW` 区分：

| `state` | 阶段 |
|---------|------|
| `ACTIVE` (3) | 即将充满：顶部细均衡 |
| `MID_RELAX` (5) | 中段：停充松弛鉴别 |
| `MID_PROTECT` (6) | 中段：泄最高芯 |
| 其它 | 未均衡（看 `inhibit_reason`） |

`delta_ok`（Δ ≤ 15 mV）只对顶部验收有意义；中段即使策略正常也常为 0。

**结构体**：`uart_battery_balance_report_t`（`uart_battery_balance_report.h`），8 B。

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | `flags` | uint8 | 见下表 |
| 1 | `state` | uint8 | `balance_state_t` |
| 2 | `inhibit_reason` | uint8 | `balance_inhibit_reason_t` |
| 3 | `mid_class` | uint8 | `balance_mid_class_t`（非中段为 0） |
| 4–5 | `delta_mv` | uint16 | 当前压差 mV（LE） |
| 6–7 | `active_mask` | uint16 | Cell1~6 泄放掩码（LE，`0x003F`） |

### `flags`

| 位 | 宏 | 含义 |
|----|-----|------|
| 0 | `BMS_BAL_FLAG_ENABLED` | 用户总开关开 |
| 1 | `BMS_BAL_FLAG_DELTA_OK` | Δ ≤ 15 mV |
| 2 | `BMS_BAL_FLAG_IMBALANCE_CHG` | 压差/中段停充 |
| 3 | `BMS_BAL_FLAG_TOP_WINDOW` | 即将充满窗口（SOC≥90% 或 vmin≥3900，或已在 ACTIVE） |
| 4 | `BMS_BAL_FLAG_BLEEDING` | 正在泄放（`ACTIVE` 或 `MID_PROTECT`） |

**数据来源**：`BmsBalanceSnapshot_Fill()` ← `Balance_GetStatus()`。

**发送**：`BMS_CanBalanceTx_Process()`；周期 **1000 ms**，`state` 或 `active_mask` 变化时 **立即重发**。不因 `delta_mv` 抖动连发。

同一快照还走：

| 通道 | 方式 |
|------|------|
| CAN `0x49B` | 周期 + 变化即发 |
| LIN PID **0x33** | Master 轮询，Slave 应答 8 B（握手后） |
| RTT | `BmsBalanceRtt_Process()`，周期与变化同 CAN；J-Link RTT Viewer |

LIN 应答布局与上表完全相同（无分片、无 `msg_type`）。充电桩需调度 PID `0x33` 才会出帧。

RTT 一行示例：`BAL top TOP d=42 msk=0x05 inh=0 mid=0 f=0x19 BLEED`

---

## 0x441 — SET_CHARGE_CTRL（TYPE 0x41，407 → BMS）

**用途**：底盘经 CAN 远程设充电电流、开始/停止充电。开充走 BMS `ChargeGate_Evaluate()`；失败不进入 CHARGING，原因见 `ChargeManager_GetLastReject()` / **0x4A1**。

**结构体**：`uart_charge_ctrl_cmd_t`（`uart_charge_ctrl.h`）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | `cmd` | uint8 | `0` 仅设电流；`1` 开始；`2` 停止 |
| 1–2 | `i_target_ma` | uint16 LE | 目标 mA：400–3000，50 mA 步进；STOP 必须为 0 |

**407 侧**：UART `SET_CHARGE_CTRL (0x41)` → `BMS_Can_SendChargeCtrl()` 发 CAN **0x441**（单帧，DLC 5）。

**BMS 侧**：`bms_can_rx.c` 收帧 → `LinCharger_ApplyCanCommand()` 更新 `target_charge_ma`；START 调用 `ChargeManager_RequestStart(true)`。LIN 状态轮询里 `i_allow_ma = min(target, 保护上限, 充电桩 VI 能力)` → 充电桩跟流。

**联调**：PC 串口连 407（CDC/UART6），发完整 UART 帧；600 mA 开始示例见 PawDrive `docs/UART_PROTOCOL.md` §10.5。

## 0x4A0 / 0x4A1 — 充电命令与安全仲裁应答

**流程**：用户（ROS）发开充/停充 → BMS 安全仲裁 → 通过则充电；失败则 **0xA1 带回主因 `reject_code` + `inhibit_mask`**。停充不仲裁，立即 `Stop()`。

ROS 开充要求 LIN 已完成 V/I 协商（`session >= VI_OK`）且未丢帧。

### 0x4A0 CMD（底板 → BMS，单帧 8 B）

| 偏移 | 字段 | 说明 |
|------|------|------|
| 0 | `cmd` | 0=STOP，1=START，2=QUERY |
| 1 | `seq` | 预留 |
| 2–3 | `request_id` | uint16 LE，幂等/配对 |
| 4–7 | 预留 | 0 |

### 0x4A1 RSP（BMS → 底板，单帧 8 B）

| 偏移 | 字段 | 说明 |
|------|------|------|
| 0 | `cmd` | 回显 |
| 1 | `flags` | bit0 `accepted`，bit1 `charging`，bit2 `paused` |
| 2–3 | `request_id` | 回显 LE |
| 4 | `reject_code` | `charge_reject_t`，accepted 时为 0 |
| 5 | `state` | `charge_state_t` |
| 6–7 | `inhibit_mask` | LE，`CHG_INH_*` |

`reject_code` 见 `User_APP/inc/charge_reject.h`。ROS 接口见 **[ROS_CHARGE.md](ROS_CHARGE.md)**。

---

## 联调调试帧（可选，默认关闭）

编译时 `-DBMS_CAN_DEBUG=ON`（CMake `option(BMS_CAN_DEBUG)`）启用 `0x48C`–`0x48F`，周期与 0x48B 相同（5 Hz）。

| CAN ID | 内容（每帧 8 字节，无分片） |
|--------|------------------------------|
| **0x48C** | flags, meas_fail, temp_fail, pack_mV(LE), CC2_mA(LE), eff/100 |
| **0x48D** | output_mV(LE), cell1~3 mV(LE) |
| **0x48E** | cell4~6 mV(LE), TS1 ×0.1℃(LE s16) |
| **0x48F** | TS2 ×0.1℃, CC3 mA(LE)，其余填 0 |

`0x48C flags`：bit0 `meas.valid`，bit1 `temp.valid`，bit2 0x48B 有效，bit3 `BQ76942_IsReady`。

---

## 软件模块（BMS 侧）

| 模块 | 文件 | 职责 |
|------|------|------|
| 传输常量 | `User_APP/inc/can_uart_transport.h` | CAN ID、分片长度 |
| 0x8B 结构 | `User_APP/inc/uart_battery_report.h` | payload 定义 |
| 0x9A 结构 | `User_APP/inc/uart_battery_ext_report.h` | 扩展 payload、告警宏 |
| 0x9B 结构 | `User_APP/inc/uart_battery_balance_report.h` | 均衡监控 payload |
| 主包快照 | `User_APP/src/bms_data_snapshot.c` | BQ → 0x8B |
| 扩展快照 | `User_APP/src/bms_ext_snapshot.c` | 告警 + 扩展 → 0x9A |
| 均衡快照 | `User_APP/src/bms_balance_snapshot.c` | `Balance_GetStatus()` → 0x9B |
| CAN 发送 | `User_APP/src/bms_can_tx.c` | 0x48B |
| CAN 接收 | `User_APP/src/bms_can_rx.c` | **0x441** 充电控制 |
| CAN 扩展发送 | `User_APP/src/bms_can_ext_tx.c` | 0x49A |
| CAN 均衡发送 | `User_APP/src/bms_can_balance_tx.c` | 0x49B |
| 均衡 RTT | `User_APP/src/bms_balance_rtt.c` | J-Link RTT 文本 |
| CAN 充电命令 | `User_APP/src/bms_can_charge_cmd.c` | 0x4A0 收、0x4A1 回；过滤表 |
| 充电仲裁 | `User_APP/src/charge_gate.c` | 开充前只读闸门 |
| LIN 充电/均衡 | `User_APP/src/lin_charger.c` | PID 0x32 充电状态；**0x33** 均衡 8 B |
| 调试发送 | `User_APP/src/bms_can_debug.c` | 0x48C–0x48F |
| 任务 | `User_APP/src/app_freertos.c` | `CommTask` 调用发送 |

初始化：`main.c` 中 `BMS_CanChargeCmd_Init()`（过滤 0x441 + 0x4A0）后 `HAL_FDCAN_Start()`；`BMS_CanRx_Init()` 开 RX FIFO 中断。`CommTask` 调用 `BMS_CanRx_Process()` 与 `BMS_CanChargeCmd_Process()`。

---

## 底盘主控（407）对接状态

| 项目 | 状态 |
|------|------|
| 接收 0x48B / TYPE 0x8B | 已有（`PawDrive-Base-Controller`） |
| 接收 0x49A / TYPE 0x9A | **待同步**（`uart_protocol.h`、`bms_can_task.c`） |
| 接收 0x49B / TYPE 0x9B | **待同步**（均衡监控） |
| 充电命令 0x4A0 / 应答 0x4A1 | **待同步**（ROS Service 见 `docs/ROS_CHARGE.md`） |

---

## PCAN 验证要点

上电约 1.5 s 后：

1. **0x48B**：每 200 ms 连续 **4** 帧，`frag_total = 04`。
2. **0x49A**：约每 1 s **6** 帧，`frag_total = 06`；告警变化时额外 burst。
3. **0x49B**：约每 1 s **2** 帧，`frag_total = 02`；`state` / `active_mask` 变化时额外 burst。
4. 量产固件不应出现 **0x48C**–**0x48F**（除非开启 `BMS_CAN_DEBUG`）。

解码示例（0x48B 电压）：重组后 offset 4–7 为 float LE，如 `33 33 C1 41` ≈ 24.15 V。

---

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v0.5 | 2026-08 | 新增 **407 → BMS** `0x441`（UART `0x41`）充电控制；FDCAN RX + `target_charge_ma` |
| v0.4 | 2026-08 | 新增 **0x49B**（TYPE 0x9B）均衡监控；LIN PID 0x33；RTT 打印；0x49A 增加 BALANCING / DELTA_HIGH |
| v0.3 | 2026-03 | 新增 **0x49A**（TYPE 0x9A）告警与扩展测量；BMS 文档初版 |
| v0.2 | — | 弃用自定义 0x180/0x181，改为 UART 0x8B 分片 → 0x48B |
| v0.1 | — | （已废弃）自定义 CAN 应用协议 |
