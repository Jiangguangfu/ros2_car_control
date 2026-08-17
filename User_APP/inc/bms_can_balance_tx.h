/**
 * @file    bms_can_balance_tx.h
 * @brief   UART 0x9B → CAN 0x49B 均衡监控（1 Hz，state/mask 变化即发）
 */
#ifndef BMS_CAN_BALANCE_TX_H
#define BMS_CAN_BALANCE_TX_H

#ifdef __cplusplus
extern "C" {
#endif

void BMS_CanBalanceTx_Init(void);
void BMS_CanBalanceTx_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* BMS_CAN_BALANCE_TX_H */
