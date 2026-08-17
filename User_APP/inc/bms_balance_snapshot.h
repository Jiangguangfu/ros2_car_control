/**
 * @file    bms_balance_snapshot.h
 * @brief   Balance_GetStatus() → uart_battery_balance_report_t
 */
#ifndef BMS_BALANCE_SNAPSHOT_H
#define BMS_BALANCE_SNAPSHOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uart_battery_balance_report.h"

void BmsBalanceSnapshot_Fill(uart_battery_balance_report_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BMS_BALANCE_SNAPSHOT_H */
