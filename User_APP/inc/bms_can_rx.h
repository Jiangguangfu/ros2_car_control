/**
 * @file    bms_can_rx.h
 * @brief   FDCAN RX：407 → BMS 充电控制（CAN 0x441）
 */
#ifndef BMS_CAN_RX_H
#define BMS_CAN_RX_H

#ifdef __cplusplus
extern "C" {
#endif

void BMS_CanRx_Init(void);
void BMS_CanRx_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* BMS_CAN_RX_H */
