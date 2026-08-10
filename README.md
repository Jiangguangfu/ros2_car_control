# BMS Project

STM32U385CG BMS firmware (CMake + FreeRTOS + J-Link debug).

**电池包：** 6S NMC，标称 22.2 V，满充 25.2 V（单节 4.2 V）  
**AFE：** BQ76942（I2C，被动均衡）

## Build

```bash
cmake -S . -B build/Debug -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build/Debug --target BMS_Project --parallel
```

Output: `build/Debug/BMS_Project.elf`

## CAN 通信

BMS 无对外 UART，电池数据经 **CAN** 上报底盘（500 kbit/s，FDCAN PA11/PA12）。

| CAN ID | 内容 | 周期 |
|--------|------|------|
| **0x48B** | 电池状态（电压、电流、SOC、温度） | 5 Hz |
| **0x49A** | 告警 + 扩展（单体电压、双温、双电流等） | 1 Hz |

协议与 payload 布局详见 **[docs/BMS_CAN.md](docs/BMS_CAN.md)**。

联调调试帧 `0x48C`–`0x48F`：CMake `-DBMS_CAN_DEBUG=ON`（量产默认关闭）。

## 软件架构

```
CommTask          — CAN 上报（0x48B / 0x49A，5 Hz 调度）
ServiceTask       — 空闲占位（上电时序在 main）
PowerTask         — bsp_power_rails（200 ms：热 + 过流/短路 + 多电源/风扇）
BmsTask           — BQ76942 采样 + Safety A/B/C + 均衡/充电/SOC/SOH（500 ms）
```

| 模块 | 文件 | 职责 |
|------|------|------|
| BQ 驱动 | `User_APP/src/bq76942.c` | 温度、6 节电压、电流、FET、均衡、Safety A/B/C |
| 电源保护 | `User_APP/src/bsp_power_rails.c` | 24/19/12/6.5/5V、风扇、热/OC-SC 统一策略 |
| 被动均衡 | `User_APP/src/cell_balance_manager.c` | 充电末期均衡策略与状态 |
| 充电路径 | `User_APP/src/charge_path.c` | CFETOFF/DFETOFF 仲裁（热 OR 过流 OR 压差停充 OR 充电管理） |

### 多电源与统一保护（bsp_power_rails）

对外唯一状态：`pwr_rails_status_t`（`BSP_PowerRails_GetStatus()`）。

| 来源 | 动作摘要 |
|------|----------|
| 热 NORMAL/WARN/LIMIT/FAULT | 风扇 PWM；高温 LIMIT 关 19V；FAULT 全关 + 禁充放 |
| BQ SCD / OCD | FAULT：全关电源轨 + 禁充放（锁存） |
| BQ OCC | FAULT：禁充，输出轨保持 |
| 软过流（CC2） | WARN 关 19V；FAULT 全关（锁存） |

BmsTask 读 Safety Status A/B/C 原始字节后调用 `BSP_PowerRails_UpdateBqSafety()` 缓存到状态（`status_a/b/c` + 解析 `scd/ocd/occ/bq_any`）。

故障恢复（`BSP_PowerRails_Process` 内自动，无需人工 Clear）：

| 故障 | 恢复条件 |
|------|----------|
| 过热 FAULT | Tmax ≤ 50 °C（`THERMAL_FAULT_EXIT`） |
| 传感器 FAULT | 温度读数恢复有效 |
| 低温 LIMIT | Tmin ≥ 3 °C |
| SCD / OCD / 软过流 | BQ 对应位清除且 \|I\| ≤ 2 A |
| OCC | BQ OCC 位清除（不锁存） |

## 被动均衡（cell_balance_manager）

### 功能

- 被动均衡：BQ76942 `CB_ACTIVE_CELLS`（Cell1~6，掩码 `0x003F`）
- 均衡策略：充电顶部 + 压差 + 温度 + 故障 + 采样稳定
- 状态监控：`Balance_GetStatus()`
- 开/关：`Balance_SetEnabled()`

### 设计原则

- **仅充电顶部均衡**（充电器在 + CHG FET 导通 + 非明显放电）
- **启动严、保持宽、退出敏感**（充电器/FET 断开立即停）
- **启动与保持逻辑分离**（禁止共用同一组启动条件做保持）
- **SOC 与 vmin 互斥二选一**（禁止 `high_soc || vmin≥3900`）

### 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `BMS_CELL_COUNT` | 6 | 6S |
| `BALANCE_START_DELTA_MV` | 40 | 启动压差 |
| `BALANCE_STOP_DELTA_MV` | 15 | 停止 / 验收压差 |
| `BALANCE_HARD_MIN_CELL_MV` | 3000 | 硬安全下限 |
| `BALANCE_NORMAL_MAX_CELL_MV` | 4200 | 过压禁止 |
| `BALANCE_VMIN_START_MV` | 3900 | 无 SOC 时启动 vmin |
| `BALANCE_VMIN_HOLD_MV` | 3850 | 无 SOC 时保持 vmin |
| `BALANCE_SOC_START_PERCENT` | 90 | 有 SOC 时启动 |
| `BALANCE_SOC_HOLD_PERCENT` | 88 | 有 SOC 时保持 |
| `BALANCE_MAX_CELLS_AT_ONCE` | 2 | 最多同时均衡节数 |
| `BALANCE_DISCHARGE_RECOVER_MA` | -50 | 允许继续（mA） |
| `BALANCE_DISCHARGE_EXIT_MA` | -100 | 明显放电（mA） |
| `BALANCE_SAMPLE_STABLE_COUNT` | 3 | 采样稳定次数（≈1.5 s） |
| `CHARGE_IMBALANCE_STOP_DELTA_MV` | 50 | 充电中 Δ≥50 → 停止充电 |
| `CHARGE_IMBALANCE_RESUME_DELTA_MV` | 30 | Δ≤30 → 解除压差停充 |

### 压差停充（与均衡独立）

充电中发现压差过大时先停充，被动均衡可继续，压差缩小后再恢复充电：

```
正常充电
  → Δ ≥ 50 mV：置位压差停充，拉高 CFETOFF，关闭充电路径
  → 仅保留被动均衡（不因 CHG FET 关闭而退出）
  → Δ ≤ 30 mV：清除压差停充，恢复充电（热管理未禁止时）
  → 均衡继续
  → Δ ≤ 15 mV：均衡完成
```

CFETOFF 由 `charge_path` 仲裁：`热/过流禁止 OR imbalance_charge_inhibit OR charge_manager`。

### 压差与开启均衡条件（有 SOC / 无 SOC）

**单体压差 Δ = vmax − vmin 的阈值与有无 SOC 无关**；有无 SOC 只影响是否进入「充电顶部均衡窗口」，不改变压差门槛。

#### 压差阈值（有 SOC / 无 SOC 相同）

| 项目 | 阈值 | 说明 |
|------|------|------|
| **开启均衡（启动）** | **Δ ≥ 40 mV** | `BALANCE_START_DELTA_MV`，仅未在 ACTIVE 时检查 |
| **停止均衡** | **Δ ≤ 15 mV** | `BALANCE_STOP_DELTA_MV`，保持阶段 Δ ≤ 15 即退出 |
| **保持均衡（滞回区）** | **15 mV < Δ < 40 mV** | 已在 ACTIVE → 继续；未启动 → 不启动 |
| **验收达标** | **Δ ≤ 15 mV** | `delta_ok = true` |
| **选节候选** | **Vcell ≥ vmin + 15 mV** | 至少高出最低节 15 mV 才参与均衡 |

#### 顶部窗口（有 SOC / 无 SOC 不同）

压差仍须 **Δ ≥ 40 mV** 才可启动，另需满足 `top_start_ready`：

| 条件 | **有 SOC**（`soc_valid = true`） | **无 SOC**（`soc_valid = false`） |
|------|----------------------------------|-----------------------------------|
| **启动窗口** | SOC ≥ **90%** | vmin ≥ **3900 mV** |
| **保持窗口** | SOC ≥ **88%** | vmin ≥ **3850 mV** |

#### 完整开启条件

**有 SOC：**

```
charger_active_stable && SOC >= 90% && Δ >= 40 mV && common_ok
```

| SOC | vmin | Δ | 能否开启 |
|-----|------|---|----------|
| 70% | 3950 mV | 50 mV | 否（SOC < 90%） |
| 92% | 3840 mV | 50 mV | 是（不要求 vmin ≥ 3900） |
| 92% | 4010 mV | 35 mV | 否（Δ < 40） |

**无 SOC：**

```
charger_active_stable && vmin >= 3900 mV && Δ >= 40 mV && common_ok
```

| vmin | Δ | 能否开启 |
|------|---|----------|
| 3950 mV | 50 mV | 是 |
| 3850 mV | 50 mV | 否（vmin < 3900） |
| 3950 mV | 35 mV | 否（Δ < 40） |

#### 速查

| 问题 | 答案 |
|------|------|
| 开启均衡的压差 | **≥ 40 mV**（有/无 SOC 相同） |
| 停止均衡的压差 | **≤ 15 mV**（有/无 SOC 相同） |
| 停止充电的压差 | **≥ 50 mV** |
| 恢复充电的压差 | **≤ 30 mV** |
| 有 SOC 额外条件 | SOC ≥ 90% 启动，≥ 88% 保持 |
| 无 SOC 额外条件 | vmin ≥ 3900 mV 启动，≥ 3850 mV 保持 |

### 启动 / 保持 / 退出

**公共条件 `common_ok`（安全，不含 3900 mV 业务门槛）：**

```
user_enabled
&& 采样连续稳定 3 次
&& vmin >= 3000 mV
&& vmax <= 4200 mV
&& 温度正常
&& 无 critical_fault
```

**顶部就绪（`Balance_IsTopReady`，SOC 与 vmin 二选一）：**

```
soc_valid == true  → 启动 SOC >= 90%，保持 SOC >= 88%
soc_valid == false → 启动 vmin >= 3900 mV，保持 vmin >= 3850 mV
```

**启动（`state != ACTIVE`）：**

```
start_conditions_ok =
    common_ok
    && charger_active_stable
    && top_start_ready
    && delta >= 40 mV
```

**保持（`state == ACTIVE`）：**

```
hold_conditions_ok =
    common_ok
    && charger_active
    && top_hold_ready
    && delta > 15 mV
```

**退出（任一成立立即关断）：**

```
delta <= 15 mV
|| !charger_present
|| !chg_fet_on
|| 明显放电（电流 <= -100 mA，经去抖）
|| !temperature_normal
|| critical_fault
|| !user_enabled
```

**选节：** 候选 `cell_mv[i] >= vmin + 15 mV`，取电压最高的最多 2 节。

**充电判定：** 不使用 `current >= 100 mA`；使用 `charger_present && chg_fet_on && current >= -50 mA`（施密特回差）。

### API

```c
void Balance_Init(void);
void Balance_Process(I2C_HandleTypeDef *hi2c);   /* BmsTask 500 ms */

void Balance_SetEnabled(bool enable);
bool Balance_IsEnabled(void);

void Balance_SetChargerPresent(bool present);    /* P1：专用充电器 GPIO */
void Balance_SetSoc(uint8_t soc_percent, bool valid); /* P2：SOC 模块 */

const balance_status_t *Balance_GetStatus(void);
```

### 状态

| 状态 | 说明 |
|------|------|
| `DISABLED` | 用户关闭 |
| `IDLE` | 监控中，未均衡 |
| `WAIT_CHARGE` | 充电路径未 stable |
| `ACTIVE` | 正在均衡（≤2 节） |
| `INHIBITED` | 安全 / 故障禁止 |

### 后续对接

| 阶段 | 内容 |
|------|------|
| P1 | `Balance_SetChargerPresent()` 接硬件；Battery Status 解析 |
| P2 | `Balance_SetSoc()` 接 SOC 算法；CommTask 状态上报 |

未调用 `Balance_SetChargerPresent()` 时，默认以 `chg_fet_on` 推断充电器在。  
未调用 `Balance_SetSoc(..., true)` 时，使用 vmin 3900/3850 mV 回退判据。

## 目录结构（User_APP）

```
docs/
  BMS_CAN.md              — CAN 协议说明（0x48B / 0x49A）
User_APP/
  inc/
    can_uart_transport.h
    uart_battery_report.h
    uart_battery_ext_report.h
    bms_can_tx.h
    bms_can_ext_tx.h
    bq76942.h
    bsp_power_rails.h
    cell_balance_manager.h
    charge_path.h
    charge_manager.h
    app_freertos.h
  src/
    bms_can_tx.c
    bms_can_ext_tx.c
    bms_data_snapshot.c
    bms_ext_snapshot.c
    bq76942.c
    bsp_power_rails.c
    cell_balance_manager.c
    charge_path.c
    charge_manager.c
    app_freertos.c
```
