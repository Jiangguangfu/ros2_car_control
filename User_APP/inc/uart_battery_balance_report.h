/**
 * @file    uart_battery_balance_report.h
 * @brief   UART 0x9B REPORT_BATTERY_BALANCE payload（CAN 0x49B）
 *
 * Compact monitor view. Do not dump balance_status_t.
 * Numeric state / inhibit / mid_class match cell_balance_manager.h enums.
 */
#ifndef UART_BATTERY_BALANCE_REPORT_H
#define UART_BATTERY_BALANCE_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BMS_BAL_FLAG_ENABLED          (1u << 0)
#define BMS_BAL_FLAG_DELTA_OK         (1u << 1)
#define BMS_BAL_FLAG_IMBALANCE_CHG    (1u << 2)
#define BMS_BAL_FLAG_TOP_WINDOW       (1u << 3)
#define BMS_BAL_FLAG_BLEEDING         (1u << 4)

typedef struct __attribute__((packed)) {
  uint8_t  flags;
  uint8_t  state;
  uint8_t  inhibit_reason;
  uint8_t  mid_class;
  uint16_t delta_mv;
  uint16_t active_mask;
} uart_battery_balance_report_t;

#ifdef __cplusplus
}
#endif

#endif /* UART_BATTERY_BALANCE_REPORT_H */
