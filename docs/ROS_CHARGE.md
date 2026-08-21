# ROS 充电控制（底板节点）

用户发命令 → BMS 安全仲裁 → 通过则充电；失败则 Service 返回 BMS 主因。

底板将 Service 转为 CAN **0x441**（UART `0x41`，推荐）或 **0x4A0**；失败原因看 **0x4A1**（`reject_code`）。建议超时 500 ms。

## 接口

建议 Service：`/bms/charge_enable`（类型可与下列字段对齐）

```
# ChargeEnable.srv
bool enable
uint32 request_id
---
bool accepted
bool charging
bool paused
uint8 reject_code
uint16 inhibit_mask
uint8 state
```

`reject_code` 与固件 `charge_reject_t` 一致：

| 码 | 含义 |
|----|------|
| 0 | 允许 / 已执行 |
| 1 | 已满电未回落 |
| 2 | BQ 测量无效 |
| 3 | BQ 通信失败 |
| 4 | 过流/短路保护 |
| 5 | 过温/传感器 |
| 6 | 低温禁充 |
| 7 | 过压 |
| 8 | 欠压 |
| 9 | 压差停充 |
| 10 | LIN 丢帧 |
| 11 | 充电桩未就绪（未 V/I 协商或无桩） |
| 12 | 充电故障未恢复 |
| 14 | 命令已接受但超时无充电电流（接触不良 / 桩没电等） |

## 命令成功 ≠ 正在充电

Service 立刻返回的 `accepted` 只表示 BMS 仲裁通过、开关已开。`charging` 要等充电电流 ≥ 50 mA（`current_a` ≤ -0.05）。

| 阶段 | 0x4A1 flags | 0x48B reserved1 | 建议 ROS `power_supply_status` |
|------|-------------|-----------------|--------------------------------|
| ACK 后 ~10 s 内等出流 | `accepted` + `waiting` | bit1 + bit3 | 可保持 UNKNOWN，或显示「启动中」 |
| 电流已确认 | `accepted` + `charging` | bit1 + bit2 | `CHARGING` |
| 超时仍无流 | `accepted` + `no_flow`，`reject_code=14` | bit1 + bit4 | `NOT_CHARGING`，文案「开关开了但没充上」 |
| 已满电闸门拒绝 | `accepted=false`，`reject_code=1` | 无 bit1 | `FULL` / `NOT_CHARGING` |

底板应 **订阅 0x4A1 变化**（以及 0x48B 5 Hz），不要只靠用户 `echo /battery_state`。0x49A 告警位 `CHG_NO_CURRENT` 会在无流时立即上报。

## 命令示例

包名按底板实际替换（下例为 `bms_msgs`）。

开充：

```bash
ros2 service call /bms/charge_enable bms_msgs/srv/ChargeEnable "{enable: true, request_id: 1}"
```

停充：

```bash
ros2 service call /bms/charge_enable bms_msgs/srv/ChargeEnable "{enable: false, request_id: 2}"
```

只查询能否充（CAN cmd=2，若底板已实现）：

```bash
ros2 service call /bms/charge_query bms_msgs/srv/ChargeEnable "{enable: false, request_id: 3}"
```

看状态（有周期 Topic 时）：

```bash
ros2 topic echo /bms/charge_status
```

成功：`accepted: true` 只表示命令通过。正在充电还需 `charging: true`（或 0x48B bit2 / `current_a <= -0.05`）。  
若 `accepted: true` 且随后 `no_flow` / `reject_code: 14`：开关已开但没充上。  
闸门失败：`accepted: false`，读 `reject_code`。

底板未实现 Service 时，可用 CAN 工具发 0x4A0：`01 00 01 00 00 00 00 00`（START，request_id=1）。
