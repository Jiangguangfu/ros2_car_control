/**
 * @file    uart_battery_ext_report.h
 * @brief   UART 0x9A REPORT_BATTERY_EXT payload（与 PawDrive 路线 1 对齐，407 待同步）
 *
 * CAN ID = 0x400 + 0x9A = 0x49A，分片格式同 0x8B → 0x48B。
 */
#ifndef UART_BATTERY_EXT_REPORT_H
#define UART_BATTERY_EXT_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "bq76942.h"

/** 告警 severity */
#define BMS_EXT_SEVERITY_NONE     0u
#define BMS_EXT_SEVERITY_WARN     1u
#define BMS_EXT_SEVERITY_CRITICAL 2u

/** source_flags：哪些模块参与了本帧汇总 */
#define BMS_EXT_SOURCE_BQ_MEAS    (1u << 0)
#define BMS_EXT_SOURCE_THERMAL    (1u << 1)
#define BMS_EXT_SOURCE_CHARGE     (1u << 2)
#define BMS_EXT_SOURCE_BALANCE    (1u << 3)
#define BMS_EXT_SOURCE_PROTECT    (1u << 4)

/** alarm_flags */
#define BMS_EXT_ALARM_OVP              (1u << 0)
#define BMS_EXT_ALARM_UVP              (1u << 1)
#define BMS_EXT_ALARM_OCP              (1u << 2)
#define BMS_EXT_ALARM_OVERTEMP         (1u << 3)
#define BMS_EXT_ALARM_COLD_CHARGE      (1u << 4)
#define BMS_EXT_ALARM_BQ_PROTECT       (1u << 5)
#define BMS_EXT_ALARM_IMBALANCE_CHG    (1u << 6)
#define BMS_EXT_ALARM_COMM_FAIL        (1u << 7)
#define BMS_EXT_ALARM_CHG_INHIBIT      (1u << 8)
#define BMS_EXT_ALARM_DSG_INHIBIT      (1u << 9)
#define BMS_EXT_ALARM_CHARGE_FAULT     (1u << 10)
#define BMS_EXT_ALARM_SHORT_CIRCUIT    (1u << 11)
#define BMS_EXT_ALARM_LOW_BATTERY      (1u << 12)
#define BMS_EXT_ALARM_BALANCING        (1u << 13)
#define BMS_EXT_ALARM_DELTA_HIGH       (1u << 14)
#define BMS_EXT_ALARM_CHG_NO_CURRENT   (1u << 15)  /* 命令已接受但无充电电流 */

typedef struct __attribute__((packed)) {
  uint32_t alarm_flags;
  uint8_t  severity;
  uint8_t  source_flags;
  uint16_t output_mv;
  uint16_t vcell_min_mv;
  uint16_t vcell_max_mv;
  int16_t  current_cc2_ma;
  int16_t  current_cc3_ma;
  int16_t  ts1_c_x10;
  int16_t  ts2_c_x10;
  uint16_t cell_mv[BQ76942_CELL_COUNT];
} uart_battery_ext_report_t;

#ifdef __cplusplus
}
#endif

#endif /* UART_BATTERY_EXT_REPORT_H */
