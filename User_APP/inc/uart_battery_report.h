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
#define BMS_BATTERY_REPORT_VALID_BIT     (1u << 0)
/** reserved1：充电闭环（与 current_a < -0.05 对齐） */
#define BMS_BATTERY_REPORT_CHG_ENABLE    (1u << 1)  /* 命令已接受，开关开 */
#define BMS_BATTERY_REPORT_CHG_CURRENT   (1u << 2)  /* 已确认充电电流 */
#define BMS_BATTERY_REPORT_CHG_WAITING   (1u << 3)  /* ACK 后等待出流（预充窗口） */
#define BMS_BATTERY_REPORT_CHG_NO_FLOW   (1u << 4)  /* 开关开了但没充上 */

/* 0x8B REPORT_BATTERY_STATE：与 ROS2 sensor_msgs/BatteryState 常用字段子集一致
 * percentage 为 0.0f~1.0f，未知时为 -1.0f；current_a 放电为正。
 * reserved0：SOH 0~100；reserved1 bit0 数据有效；bit1 充电使能；bit2 电流已确认；bit3 等待出流；bit4 无流。
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
