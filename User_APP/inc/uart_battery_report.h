/**
 * @file    uart_battery_report.h
 * @brief   UART 0x8B REPORT_BATTERY_STATE payload（与底盘 uart_protocol.h 一致）
 */
#ifndef UART_BATTERY_REPORT_H
#define UART_BATTERY_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** BMS 电池状态 CAN 上报周期（5 Hz） */
#ifndef BMS_CAN_BATTERY_PERIOD_MS
#define BMS_CAN_BATTERY_PERIOD_MS     200u
#endif

/** reserved1 bit0：1 = BMS CAN 数据有效 */
#define BMS_BATTERY_REPORT_VALID_BIT  (1u << 0)

/* 0x8B REPORT_BATTERY_STATE：与 ROS2 sensor_msgs/BatteryState 常用字段子集一致
 * percentage 为 0.0f~1.0f，未知时为 -1.0f；current_a 放电为正。
 * reserved0：SOH 0~100；reserved1 bit0：BMS CAN 数据有效。
 */
typedef struct __attribute__((packed)) {
  uint8_t series_cells;
  uint8_t present;
  uint8_t reserved0;
  uint8_t reserved1;
  float voltage_v;
  float current_a;
  float percentage;
  float temperature_c;
} uart_battery_state_report_t;

#ifdef __cplusplus
}
#endif

#endif /* UART_BATTERY_REPORT_H */
