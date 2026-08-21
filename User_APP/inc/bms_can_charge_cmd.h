/**
 * @file    bms_can_charge_cmd.h
 * @brief   CAN 0x4A0 充电命令 / 0x4A1 仲裁应答
 */
#ifndef BMS_CAN_CHARGE_CMD_H
#define BMS_CAN_CHARGE_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

void BMS_CanChargeCmd_Init(void);

/** FDCAN Start / Bus-Off 恢复后重新装过滤表（0x441 + 0x4A0） */
void BMS_CanChargeCmd_ApplyFilters(void);

/** ISR：0x4A0 入队，CommTask 再处理 */
void BMS_CanChargeCmd_OnRx(const uint8_t *data);

/** CommTask 周期调用：收 0x4A0，回 0x4A1 */
void BMS_CanChargeCmd_Process(void);

/** 电流确认 / 无流变化时补发 0x4A1（命令成功 ≠ 正在充电） */
void BMS_CanChargeCmd_PublishStatus(void);

/** 0x441 启停后立刻推一帧 0x4A1，不等下一轮变化比较 */
void BMS_CanChargeCmd_NotifyHost(void);

#ifdef __cplusplus
}
#endif

#endif /* BMS_CAN_CHARGE_CMD_H */
