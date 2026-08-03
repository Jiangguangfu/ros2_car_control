/**
 ******************************************************************************
 * @file    thermal_manager.c
 * @brief   Thermal policy from BQ76942 TS1/TS2; fan + FET-off actuation.
 ******************************************************************************
 */
#include "thermal_manager.h"
#include "bsp_fan.h"
#include "app_freertos.h"
#include "main.h"

/* Thresholds in °C * 10. Tune after cell chemistry validation. */
#define THERMAL_WARN_ENTER_CX10           400   /* 40.0 °C */
#define THERMAL_WARN_EXIT_CX10            370   /* 37.0 °C */
#define THERMAL_LIMIT_ENTER_CX10          500   /* 50.0 °C */
#define THERMAL_LIMIT_EXIT_CX10           470   /* 47.0 °C */
#define THERMAL_FAULT_ENTER_CX10          550   /* 55.0 °C */
#define THERMAL_FAULT_EXIT_CX10           500   /* 50.0 °C */
#define THERMAL_COLD_ENTER_CX10             0   /*  0.0 °C charge inhibit */
#define THERMAL_COLD_EXIT_CX10             30   /*  3.0 °C */

#define THERMAL_FAN_DUTY_NORMAL             0U
#define THERMAL_FAN_DUTY_WARN              45U
#define THERMAL_FAN_DUTY_LIMIT             80U
#define THERMAL_FAN_DUTY_FAULT            100U

#define THERMAL_SENSOR_FAIL_THRESHOLD       6U  /* ~3 s at 500 ms sample */

static thermal_status_t s_status;
static bool s_fault_latched;

static void Thermal_SetFetOff(bool charge_off, bool discharge_off)
{
  /* BQ76942 CFETOFF/DFETOFF: host high forces corresponding FET path off. */
  HAL_GPIO_WritePin(BQ_CFETOFF_GPIO_Port, BQ_CFETOFF_Pin,
                    charge_off ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BQ_DFETOFF_GPIO_Port, BQ_DFETOFF_Pin,
                    discharge_off ? GPIO_PIN_SET : GPIO_PIN_RESET);

  s_status.charge_inhibit = charge_off;
  s_status.discharge_inhibit = discharge_off;
}

static void Thermal_ApplyActuators(void)
{
  uint8_t duty = THERMAL_FAN_DUTY_NORMAL;
  bool chg_off = false;
  bool dsg_off = false;

  switch (s_status.state)
  {
    case THERMAL_STATE_WARN:
      duty = THERMAL_FAN_DUTY_WARN;
      break;
    case THERMAL_STATE_LIMIT:
      duty = (s_status.reason == THERMAL_REASON_COLD_CHARGE) ?
             THERMAL_FAN_DUTY_NORMAL : THERMAL_FAN_DUTY_LIMIT;
      chg_off = true;
      break;
    case THERMAL_STATE_FAULT:
      duty = THERMAL_FAN_DUTY_FAULT;
      chg_off = true;
      dsg_off = true;
      break;
    case THERMAL_STATE_NORMAL:
    default:
      break;
  }

  s_status.fan_duty_percent = duty;
  BSP_Fan_SetDutyPercent(duty);
  Thermal_SetFetOff(chg_off, dsg_off);
}

static thermal_state_t Thermal_EvalHotState(int16_t tmax_c_x10, thermal_state_t prev)
{
  if (tmax_c_x10 >= THERMAL_FAULT_ENTER_CX10)
  {
    return THERMAL_STATE_FAULT;
  }

  if (prev == THERMAL_STATE_FAULT)
  {
    /* Hot fault stays latched until Thermal_ClearFault(). */
    return THERMAL_STATE_FAULT;
  }

  if (tmax_c_x10 >= THERMAL_LIMIT_ENTER_CX10)
  {
    return THERMAL_STATE_LIMIT;
  }

  if (prev == THERMAL_STATE_LIMIT)
  {
    if (tmax_c_x10 > THERMAL_LIMIT_EXIT_CX10)
    {
      return THERMAL_STATE_LIMIT;
    }
    /* fall through to warn/normal band */
  }

  if (tmax_c_x10 >= THERMAL_WARN_ENTER_CX10)
  {
    return THERMAL_STATE_WARN;
  }

  if (prev == THERMAL_STATE_WARN)
  {
    if (tmax_c_x10 > THERMAL_WARN_EXIT_CX10)
    {
      return THERMAL_STATE_WARN;
    }
  }

  return THERMAL_STATE_NORMAL;
}

void Thermal_Init(void)
{
  s_status.state = THERMAL_STATE_NORMAL;
  s_status.reason = THERMAL_REASON_NONE;
  s_status.tmax_c_x10 = 0;
  s_status.tmin_c_x10 = 0;
  s_status.die_c_x10 = 0;
  s_status.fan_duty_percent = 0U;
  s_status.charge_inhibit = false;
  s_status.discharge_inhibit = false;
  s_status.sensor_ok = false;
  s_fault_latched = false;

  BSP_Fan_Init();
  Thermal_SetFetOff(false, false);
}

void Thermal_Process(void)
{
  const bq76942_temp_t *temp = Bms_GetBqTemperatures();
  const uint32_t fail_count = Bms_GetBqTempFailCount();
  thermal_state_t next;

  if ((temp == NULL) || (!temp->valid) || (fail_count >= THERMAL_SENSOR_FAIL_THRESHOLD))
  {
    s_status.sensor_ok = false;
    s_status.state = THERMAL_STATE_FAULT;
    s_status.reason = THERMAL_REASON_SENSOR;
    s_fault_latched = true;
    Thermal_ApplyActuators();
    return;
  }

  s_status.sensor_ok = true;
  s_status.die_c_x10 = temp->int_temp_c_x10;
  s_status.tmax_c_x10 = (temp->ts1_temp_c_x10 > temp->ts2_temp_c_x10) ?
                        temp->ts1_temp_c_x10 : temp->ts2_temp_c_x10;
  s_status.tmin_c_x10 = (temp->ts1_temp_c_x10 < temp->ts2_temp_c_x10) ?
                        temp->ts1_temp_c_x10 : temp->ts2_temp_c_x10;

  /* Recover automatically from sensor FAULT once I2C reads are healthy. */
  if (s_fault_latched && (s_status.reason == THERMAL_REASON_SENSOR))
  {
    s_fault_latched = false;
    s_status.state = THERMAL_STATE_NORMAL;
    s_status.reason = THERMAL_REASON_NONE;
  }

  next = Thermal_EvalHotState(s_status.tmax_c_x10, s_status.state);

  if (next == THERMAL_STATE_FAULT)
  {
    s_fault_latched = true;
    s_status.reason = THERMAL_REASON_HOT;
  }
  else if (s_status.tmin_c_x10 < THERMAL_COLD_ENTER_CX10)
  {
    if (next < THERMAL_STATE_LIMIT)
    {
      next = THERMAL_STATE_LIMIT;
    }
    s_status.reason = THERMAL_REASON_COLD_CHARGE;
  }
  else if ((s_status.reason == THERMAL_REASON_COLD_CHARGE) &&
           (s_status.tmin_c_x10 < THERMAL_COLD_EXIT_CX10))
  {
    if (next < THERMAL_STATE_LIMIT)
    {
      next = THERMAL_STATE_LIMIT;
    }
    s_status.reason = THERMAL_REASON_COLD_CHARGE;
  }
  else if (next == THERMAL_STATE_NORMAL)
  {
    s_status.reason = THERMAL_REASON_NONE;
  }
  else
  {
    s_status.reason = THERMAL_REASON_HOT;
  }

  if (s_fault_latched && (s_status.reason == THERMAL_REASON_HOT))
  {
    next = THERMAL_STATE_FAULT;
  }

  s_status.state = next;
  Thermal_ApplyActuators();
}

const thermal_status_t *Thermal_GetStatus(void)
{
  return &s_status;
}

thermal_state_t Thermal_GetState(void)
{
  return s_status.state;
}

bool Thermal_ClearFault(void)
{
  if (s_status.state != THERMAL_STATE_FAULT)
  {
    return false;
  }

  if (!s_status.sensor_ok)
  {
    return false;
  }

  if (s_status.tmax_c_x10 > THERMAL_FAULT_EXIT_CX10)
  {
    return false;
  }

  if (s_status.reason == THERMAL_REASON_SENSOR)
  {
    return false;
  }

  s_fault_latched = false;
  s_status.state = Thermal_EvalHotState(s_status.tmax_c_x10, THERMAL_STATE_LIMIT);

  if (s_status.tmin_c_x10 < THERMAL_COLD_ENTER_CX10)
  {
    s_status.state = THERMAL_STATE_LIMIT;
    s_status.reason = THERMAL_REASON_COLD_CHARGE;
  }
  else if (s_status.state == THERMAL_STATE_NORMAL)
  {
    s_status.reason = THERMAL_REASON_NONE;
  }
  else
  {
    s_status.reason = THERMAL_REASON_HOT;
  }

  Thermal_ApplyActuators();
  return true;
}
