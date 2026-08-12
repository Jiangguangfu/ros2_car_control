/**
 * @file    bms_lin_config.h
 * @brief   BMS LIN 调试开关
 *
 * BMS_LIN_DIAG_TX_ENABLE=1：仅周期性发 LIN 测试帧（硬件/LA 排查用，非 Slave）。
 * 确认 BMS LIN 座有波形后改回 0，再烧录正常 Slave 固件。
 */
#ifndef BMS_LIN_CONFIG_H
#define BMS_LIN_CONFIG_H

#define BMS_LIN_DIAG_TX_ENABLE              0

/*
 * USART1 TX/RX Swap。PCB 上 Pin30=USART_TX(PA9) 接 LIN 电路时应为 0。
 * 与充电桩 CHARGER_LIN_USART_SWAP_ENABLE 保持一致。
 */
#define BMS_LIN_USART_SWAP_ENABLE           0

#endif /* BMS_LIN_CONFIG_H */
