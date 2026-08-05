/**
 * @file    bms_can_debug.h
 * @brief   BQ 采样调试帧（CAN），PCAN 可直接抓，无需串口
 *
 *   0x48C — 状态 + pack 电压 + CC2 电流 + eff/100
 *   0x48D — output 电压 + cell1..3 (mV, LE)
 *   0x48E — cell4..6 + TS1 (0.1°C, LE s16)
 *   0x48F — TS2 + CC3 电流
 *
 * 0x48C flags: bit0 meas.valid bit1 temp.valid bit2 CAN valid bit3 BQ76942_IsReady
 */
#ifndef BMS_CAN_DEBUG_H
#define BMS_CAN_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/** 联调时在 CMake 加 -DBMS_CAN_DEBUG=ON，或编译定义 BMS_CAN_DEBUG_ENABLE=1 */
#ifndef BMS_CAN_DEBUG_ENABLE
#define BMS_CAN_DEBUG_ENABLE  0
#endif

void BMS_CanDebug_Init(void);
void BMS_CanDebug_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* BMS_CAN_DEBUG_H */
