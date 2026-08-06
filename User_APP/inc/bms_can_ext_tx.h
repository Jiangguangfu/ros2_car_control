/**
 * @file    bms_can_ext_tx.h
 * @brief   UART 0x9A → CAN 0x49A 扩展包（告警 + 明细测量，1 Hz）
 */
#ifndef BMS_CAN_EXT_TX_H
#define BMS_CAN_EXT_TX_H

#ifdef __cplusplus
extern "C" {
#endif

void BMS_CanExtTx_Init(void);
void BMS_CanExtTx_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* BMS_CAN_EXT_TX_H */
