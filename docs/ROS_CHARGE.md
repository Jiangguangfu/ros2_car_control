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
| 13 | BQ Safety 锁存 |

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

成功：`accepted: true` 且 `charging: true`。  
失败：`accepted: false`，读 `reject_code`（BMS 仲裁原因，不是底板自己编的）。

底板未实现 Service 时，可用 CAN 工具发 0x4A0：`01 00 01 00 00 00 00 00`（START，request_id=1）。
