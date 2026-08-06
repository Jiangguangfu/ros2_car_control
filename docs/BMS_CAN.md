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

**severity**：有关键告警（BQ 保护、充电故障、过温）时为 **CRITICAL(2)**，其余为 **WARN(1)**。

**数据来源**：`BmsExtSnapshot_Fill()` ← BQ + 热管理 + 充电 + 均衡 + BQ Safety。

**发送**：`BMS_CanExtTx_Process()`；周期 **1000 ms**，`alarm_flags` 变化时 **立即重发**。

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
| 主包快照 | `User_APP/src/bms_data_snapshot.c` | BQ → 0x8B |
| 扩展快照 | `User_APP/src/bms_ext_snapshot.c` | 告警 + 扩展 → 0x9A |
| CAN 发送 | `User_APP/src/bms_can_tx.c` | 0x48B |
| CAN 扩展发送 | `User_APP/src/bms_can_ext_tx.c` | 0x49A |
| 调试发送 | `User_APP/src/bms_can_debug.c` | 0x48C–0x48F |
| 任务 | `User_APP/src/app_freertos.c` | `CommTask` 调用发送 |

初始化：`main.c` 中 `BMS_CanTx_Init()`、`BMS_CanExtTx_Init()`；FDCAN 启动后 `CommTask` 延迟 1.5 s 再发（等电源时序）。

---

## 底盘主控（407）对接状态

| 项目 | 状态 |
|------|------|
| 接收 0x48B / TYPE 0x8B | 已有（`PawDrive-Base-Controller`） |
| 接收 0x49A / TYPE 0x9A | **待同步**（`uart_protocol.h`、`bms_can_task.c`） |
| 文档 | 底盘 `docs/BMS_CAN.md` 需补充 0x49A |

---

## PCAN 验证要点

上电约 1.5 s 后：

1. **0x48B**：每 200 ms 连续 **4** 帧，`frag_total = 04`。
2. **0x49A**：约每 1 s **6** 帧，`frag_total = 06`；告警变化时额外 burst。
3. 量产固件不应出现 **0x48C**–**0x48F**（除非开启 `BMS_CAN_DEBUG`）。

解码示例（0x48B 电压）：重组后 offset 4–7 为 float LE，如 `33 33 C1 41` ≈ 24.15 V。

---

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v0.3 | 2026-03 | 新增 **0x49A**（TYPE 0x9A）告警与扩展测量；BMS 文档初版 |
| v0.2 | — | 弃用自定义 0x180/0x181，改为 UART 0x8B 分片 → 0x48B |
| v0.1 | — | （已废弃）自定义 CAN 应用协议 |
