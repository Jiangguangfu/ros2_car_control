/**
 * @file    bms_ext_snapshot.h
 * @brief   告警 + 扩展测量 → uart_battery_ext_report_t
 */
#ifndef BMS_EXT_SNAPSHOT_H
#define BMS_EXT_SNAPSHOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uart_battery_ext_report.h"

void BmsExtSnapshot_Fill(uart_battery_ext_report_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BMS_EXT_SNAPSHOT_H */
