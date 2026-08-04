/**
 * @file    bms_can_tx.h
 * @brief   FDCAN 发送 UART 0x8B 电池状态（CAN ID 0x48B，4 帧分片）
 */
#ifndef BMS_CAN_TX_H
#define BMS_CAN_TX_H

#ifdef __cplusplus
extern "C" {
#endif

void BMS_CanTx_Init(void);
void BMS_CanTx_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* BMS_CAN_TX_H */
