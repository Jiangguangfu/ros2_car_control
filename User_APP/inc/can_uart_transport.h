/**
 * @file    can_uart_transport.h
 * @brief   UART 协议 payload 经 CAN 分片传输（与底盘 PawDrive-Base-Controller 一致）
 *
 * CAN ID = CAN_UART_ID_BASE + UART TYPE（如 0x8B → 0x48B）
 * 每帧 Data[8]: [frag_idx][frag_total][6B payload 片段]
 */
#ifndef CAN_UART_TRANSPORT_H
#define CAN_UART_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CAN_UART_ID_BASE              0x400u
#define CAN_UART_FRAG_DATA_BYTES      6u

/** UART TYPE_REPORT_BATTERY_STATE */
#define CAN_UART_MSG_BATTERY          0x8Bu
#define CAN_UART_ID_BATTERY           (CAN_UART_ID_BASE + CAN_UART_MSG_BATTERY)

#define CAN_UART_BATTERY_PAYLOAD_LEN    20u
#define CAN_UART_BATTERY_FRAG_TOTAL \
    ((CAN_UART_BATTERY_PAYLOAD_LEN + CAN_UART_FRAG_DATA_BYTES - 1u) / CAN_UART_FRAG_DATA_BYTES)

#ifdef __cplusplus
}
#endif

#endif /* CAN_UART_TRANSPORT_H */
