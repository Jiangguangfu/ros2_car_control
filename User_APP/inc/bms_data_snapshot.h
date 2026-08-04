/**
 * @file    bms_data_snapshot.h
 * @brief   聚合 BQ76942 / 热管理采样 → UART 0x8B 电池状态 payload
 */
#ifndef BMS_DATA_SNAPSHOT_H
#define BMS_DATA_SNAPSHOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uart_battery_report.h"

void BmsDataSnapshot_Fill(uart_battery_state_report_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BMS_DATA_SNAPSHOT_H */
