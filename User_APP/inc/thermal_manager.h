/**
 ******************************************************************************
 * @file    thermal_manager.h
 * @brief   BMS board thermal management using BQ76942 TS1/TS2 temperatures.
 *
 * Levels (cell/pack NTC, °C*10, with hysteresis):
 *   NORMAL → WARN → LIMIT → FAULT
 * Actuators: multi-rail outputs (24/19/12/6.5/5 V), SYS_FAN_PWM;
 *            FET-off via charge_path (OR with imbalance).
 ******************************************************************************
 */
#ifndef THERMAL_MANAGER_H
#define THERMAL_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  THERMAL_STATE_NORMAL = 0,
  THERMAL_STATE_WARN,
  THERMAL_STATE_LIMIT,
  THERMAL_STATE_FAULT
} thermal_state_t;

typedef enum
{
  THERMAL_REASON_NONE = 0,
  THERMAL_REASON_HOT,
  THERMAL_REASON_COLD_CHARGE,
  THERMAL_REASON_SENSOR
} thermal_reason_t;

typedef struct
{
  thermal_state_t state;
  thermal_reason_t reason;
  int16_t tmax_c_x10;      /* max(TS1, TS2) */
  int16_t tmin_c_x10;      /* min(TS1, TS2) */
  int16_t die_c_x10;       /*内部芯片温度*/
  uint8_t fan_duty_percent;/*风扇pwm占空比*/
  uint8_t power_rails_mask;/* 已开启电源轨位掩码 (bsp_power_rails) */
  bool charge_inhibit;     /* CFETOFF asserted 充电禁止*/
  bool discharge_inhibit;  /* DFETOFF asserted 放电禁止*/
  bool sensor_ok;          /*传感器是否正常*/
} thermal_status_t;

void Thermal_Init(void);
/** Evaluate BQ temps and drive fan / FET-off pins. Call from PowerTask. */
void Thermal_Process(void);

const thermal_status_t *Thermal_GetStatus(void);
thermal_state_t Thermal_GetState(void);

/** Clear latched FAULT if temperatures have recovered. Returns true if cleared. */
bool Thermal_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_MANAGER_H */
