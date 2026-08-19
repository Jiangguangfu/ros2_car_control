/**
 ******************************************************************************
 * @file    bsp_power_rails.c
 * @brief   Multi-rail GPIO + unified thermal / BQ OC-SC / soft OCD protect.
 ******************************************************************************
 */
#include "bsp_power_rails.h"
#include "bsp_adc_rails.h"
#include "bsp_fan.h"
#include "bq76942.h"
#include "charge_path.h"
#include "charge_manager.h"
#include "lin_charger.h"
#include "app_freertos.h"
#include "main.h"
#include "cmsis_os2.h"

/* Thermal thresholds (°C * 10). */
#define THERMAL_WARN_ENTER_CX10           400
#define THERMAL_WARN_EXIT_CX10            370
#define THERMAL_LIMIT_ENTER_CX10          500
#define THERMAL_LIMIT_EXIT_CX10           470
#define THERMAL_FAULT_ENTER_CX10          550
#define THERMAL_FAULT_EXIT_CX10           500
#define THERMAL_COLD_ENTER_CX10             0
#define THERMAL_COLD_EXIT_CX10             30

#define THERMAL_FAN_DUTY_NORMAL             0U
#define THERMAL_FAN_DUTY_WARN              45U
#define THERMAL_FAN_DUTY_LIMIT             80U
#define THERMAL_FAN_DUTY_FAULT            100U
#define CHARGE_ACTIVE_FAN_DUTY             65U
#define CHARGE_FAN_MIN_CURRENT_MA            50
#define THERMAL_SENSOR_FAIL_THRESHOLD       6U

/* Soft discharge OC (mA). */
#define PROTECT_SOFT_OCD_WARN_MA          12000
#define PROTECT_SOFT_OCD_FAULT_MA         15000
#define PROTECT_SOFT_OCD_WARN_DEBOUNCE       3U
#define PROTECT_SOFT_OCD_FAULT_DEBOUNCE      2U
#define PROTECT_SOFT_CLEAR_MA             2000

#define PWR_RAILS_ENABLE_ALL_EXCEPT_19V \
  ((uint8_t)(PWR_MASK_ALL & ~PWR_MASK_19V))

/* 19V 无电压分压，到位不看电流：EN 拉高后延时，再确认无短路。 */
#define PWR_RAIL_19V_ON_DELAY_MS            500U
/* INA180 满量程约 6.6 A，持续高于此视为 19V 输出短路。 */
#define PWR_RAIL_19V_SHORT_MA               5000U

/* FAULT: 全部电源轨关闭。 */
#define PWR_RAILS_FAULT_FAN_ENABLE \
  ((uint8_t)0U)

/* Legacy alias (enable mask, not drop list). */
#define PWR_RAILS_DROP_19V  PWR_RAILS_ENABLE_ALL_EXCEPT_19V

typedef enum
{
  PWR_REQ_THERMAL = 0,
  PWR_REQ_PROTECT,
  PWR_REQ_COUNT
} pwr_req_source_t;

static pwr_rails_status_t s_pwr_rails_status;
static uint8_t s_request_mask[PWR_REQ_COUNT];
static bool s_thermal_latched;
static bool s_current_latched;
static pwr_state_t s_thermal_state;
static pwr_reason_t s_thermal_reason;
static pwr_state_t s_current_state;
static pwr_reason_t s_current_reason;
static uint8_t s_soft_warn_count;
static uint8_t s_soft_fault_count;
static bool s_boot_complete;

/* -------------------------------------------------------------------------- */
/* GPIO                                                                       */
/* -------------------------------------------------------------------------- */

static void PowerRails_Hold24VOff(void)
{
  HAL_GPIO_WritePin(PWR_24V_BYPASS_EN_GPIO_Port, PWR_24V_BYPASS_EN_Pin,
                    GPIO_PIN_RESET);
  s_pwr_rails_status.rail_on[PWR_RAIL_24V] = false;
}

static void PowerRails_WriteGpio(pwr_rail_id_t rail, bool on)
{
  GPIO_PinState level = on ? GPIO_PIN_SET : GPIO_PIN_RESET;

  switch (rail)
  {
    case PWR_RAIL_24V:
      HAL_GPIO_WritePin(PWR_24V_BYPASS_EN_GPIO_Port, PWR_24V_BYPASS_EN_Pin,
                        level);
      break;
    case PWR_RAIL_19V:
      HAL_GPIO_WritePin(PWR_19V_EN_GPIO_Port, PWR_19V_EN_Pin, level);
      break;
    case PWR_RAIL_12V:
      HAL_GPIO_WritePin(PER_12V_EN_GPIO_Port, PER_12V_EN_Pin, level);
      break;
    case PWR_RAIL_6V5:
      HAL_GPIO_WritePin(PWR_7V5_EN_GPIO_Port, PWR_7V5_EN_Pin, level);
      break;
    case PWR_RAIL_5V:
    default:
      break;
  }
}

static void PowerRails_DriveMask(uint8_t enable_mask)
{
  uint8_t i;
  bool on_6v5 = ((enable_mask & PWR_MASK_6V5) != 0U);

  for (i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    bool on;

    if (i == (uint8_t)PWR_RAIL_5V)
    {
      on = on_6v5 && ((enable_mask & PWR_MASK_5V) != 0U);
      s_pwr_rails_status.rail_on[i] = on;
      continue;
    }

    on = ((enable_mask & (1u << i)) != 0U);
    s_pwr_rails_status.rail_on[i] = on;
    PowerRails_WriteGpio((pwr_rail_id_t)i, on);
  }

  s_pwr_rails_status.enabled_mask = 0U;
  for (i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    if (s_pwr_rails_status.rail_on[i])
    {
      s_pwr_rails_status.enabled_mask =
          (uint8_t)(s_pwr_rails_status.enabled_mask | (1u << i));
    }
  }
}

static void PowerRails_SetRequest(pwr_req_source_t source, uint8_t enable_mask)
{
  if ((uint8_t)source < (uint8_t)PWR_REQ_COUNT)
  {
    s_request_mask[source] = (uint8_t)(enable_mask & PWR_MASK_ALL);
  }
}

static void PowerRails_ApplyRequests(void)
{
  uint8_t mask = PWR_MASK_ALL;
  uint8_t i;

  for (i = 0U; i < (uint8_t)PWR_REQ_COUNT; i++)
  {
    mask = (uint8_t)(mask & s_request_mask[i]);
  }

  s_pwr_rails_status.power_rails_mask = mask;
  PowerRails_DriveMask(mask);
}

/* -------------------------------------------------------------------------- */
/* Thermal evaluation                                                         */
/* -------------------------------------------------------------------------- */

static bool PowerRails_TsValid(int16_t t_c_x10)
{
  return (t_c_x10 > -400) && (t_c_x10 < 1250);
}

static void PowerRails_UpdateCellTemps(const bq76942_temp_t *temp)
{
  bool ts1_ok;
  bool ts2_ok;

  ts1_ok = PowerRails_TsValid(temp->ts1_temp_c_x10);
  ts2_ok = PowerRails_TsValid(temp->ts2_temp_c_x10);

  if (ts1_ok && ts2_ok)
  {
    s_pwr_rails_status.tmax_c_x10 = (temp->ts1_temp_c_x10 > temp->ts2_temp_c_x10) ?
                          temp->ts1_temp_c_x10 : temp->ts2_temp_c_x10;
    s_pwr_rails_status.tmin_c_x10 = (temp->ts1_temp_c_x10 < temp->ts2_temp_c_x10) ?
                          temp->ts1_temp_c_x10 : temp->ts2_temp_c_x10;
  }
  else if (ts1_ok)
  {
    s_pwr_rails_status.tmax_c_x10 = temp->ts1_temp_c_x10;
    s_pwr_rails_status.tmin_c_x10 = temp->ts1_temp_c_x10;
  }
  else if (ts2_ok)
  {
    s_pwr_rails_status.tmax_c_x10 = temp->ts2_temp_c_x10;
    s_pwr_rails_status.tmin_c_x10 = temp->ts2_temp_c_x10;
  }
  else
  {
    /* Invalid/unconnected NTC (0 K → -2732): use die temperature. */
    s_pwr_rails_status.tmax_c_x10 = temp->int_temp_c_x10;
    s_pwr_rails_status.tmin_c_x10 = temp->int_temp_c_x10;
  }
}

static pwr_state_t PowerRails_EvalHotState(int16_t tmax_c_x10, pwr_state_t prev)
{
  if (tmax_c_x10 >= THERMAL_FAULT_ENTER_CX10)
  {
    return PWR_STATE_FAULT;
  }

  if (prev == PWR_STATE_FAULT)
  {
    return PWR_STATE_FAULT;
  }

  if (tmax_c_x10 >= THERMAL_LIMIT_ENTER_CX10)
  {
    return PWR_STATE_LIMIT;
  }

  if ((prev == PWR_STATE_LIMIT) && (tmax_c_x10 > THERMAL_LIMIT_EXIT_CX10))
  {
    return PWR_STATE_LIMIT;
  }

  if (tmax_c_x10 >= THERMAL_WARN_ENTER_CX10)
  {
    return PWR_STATE_WARN;
  }

  if ((prev == PWR_STATE_WARN) && (tmax_c_x10 > THERMAL_WARN_EXIT_CX10))
  {
    return PWR_STATE_WARN;
  }

  return PWR_STATE_NORMAL;
}

static void PowerRails_EvalThermal(void)
{
  const bq76942_temp_t *temp = Bms_GetBqTemperatures();
  const uint32_t fail_count = Bms_GetBqTempFailCount();
  pwr_state_t prev;
  pwr_state_t next;

  if (temp == NULL)
  {
    return;
  }

  if (fail_count >= THERMAL_SENSOR_FAIL_THRESHOLD)
  {
    s_pwr_rails_status.sensor_ok = false;
    s_thermal_state = PWR_STATE_FAULT;
    s_thermal_reason = PWR_REASON_SENSOR;
    s_thermal_latched = true;
    return;
  }

  if (!temp->valid)
  {
    return;
  }

  s_pwr_rails_status.sensor_ok = true;
  s_pwr_rails_status.die_c_x10 = temp->int_temp_c_x10;
  PowerRails_UpdateCellTemps(temp);

  /* Sensor fault: auto-recover once reads are healthy. */
  if (s_thermal_latched && (s_thermal_reason == PWR_REASON_SENSOR))
  {
    s_thermal_latched = false;
    s_thermal_state = PWR_STATE_NORMAL;
    s_thermal_reason = PWR_REASON_NONE;
  }

  /* Hot fault: auto-recover when cooled to FAULT exit threshold. */
  if (s_thermal_latched && (s_thermal_reason == PWR_REASON_HOT) &&
      (s_pwr_rails_status.tmax_c_x10 <= THERMAL_FAULT_EXIT_CX10))
  {
    s_thermal_latched = false;
  }

  prev = s_thermal_state;
  if ((!s_thermal_latched) && (prev == PWR_STATE_FAULT))
  {
    /* Allow EvalHotState to leave FAULT after latch cleared. */
    prev = PWR_STATE_LIMIT;
  }

  next = PowerRails_EvalHotState(s_pwr_rails_status.tmax_c_x10, prev);

  if (next == PWR_STATE_FAULT)
  {
    s_thermal_latched = true;
    s_thermal_reason = PWR_REASON_HOT;
  }
  else if (s_pwr_rails_status.tmin_c_x10 < THERMAL_COLD_ENTER_CX10)
  {
    if (next < PWR_STATE_LIMIT)
    {
      next = PWR_STATE_LIMIT;
    }
    s_thermal_reason = PWR_REASON_COLD_CHARGE;
  }
  else if (s_thermal_reason == PWR_REASON_COLD_CHARGE)
  {
    if (s_pwr_rails_status.tmin_c_x10 >= THERMAL_COLD_EXIT_CX10)
    {
      s_thermal_reason = PWR_REASON_NONE;
      next = PowerRails_EvalHotState(s_pwr_rails_status.tmax_c_x10, PWR_STATE_NORMAL);
    }
    else
    {
      if (next < PWR_STATE_LIMIT)
      {
        next = PWR_STATE_LIMIT;
      }
    }
  }
  else if (next == PWR_STATE_NORMAL)
  {
    s_thermal_reason = PWR_REASON_NONE;
  }
  else
  {
    s_thermal_reason = PWR_REASON_HOT;
  }

  /* Hold hot FAULT until cooled (auto-recover above). */
  if (s_thermal_latched && (s_thermal_reason == PWR_REASON_HOT))
  {
    next = PWR_STATE_FAULT;
  }

  s_thermal_state = next;
}

/* -------------------------------------------------------------------------- */
/* Current / BQ OC-SC evaluation                                              */
/* -------------------------------------------------------------------------- */

static bool PowerRails_CurrentClearOk(void)
{
  const bq76942_meas_t *meas;
  int16_t i_abs;

  if (!s_pwr_rails_status.bq_valid)
  {
    return false;
  }

  if (s_pwr_rails_status.scd || s_pwr_rails_status.ocd || s_pwr_rails_status.occ)
  {
    return false;
  }

  meas = Bms_GetBqMeasurements();
  if ((meas == NULL) || (!meas->valid))
  {
    return false;
  }

  i_abs = (meas->current_ma < 0) ?
          (int16_t)(-meas->current_ma) : meas->current_ma;
  return (i_abs <= PROTECT_SOFT_CLEAR_MA);
}

static void PowerRails_EvalCurrent(void)
{
  const bq76942_meas_t *meas = Bms_GetBqMeasurements();
  int16_t i_ma = 0;
  int16_t i_abs;
  pwr_state_t next = PWR_STATE_NORMAL;
  pwr_reason_t reason = PWR_REASON_NONE;

  if ((meas != NULL) && meas->valid)
  {
    i_ma = meas->current_ma;
  }
  s_pwr_rails_status.pack_current_ma = i_ma;
  i_abs = (i_ma < 0) ? (int16_t)(-i_ma) : i_ma;

  /* SCD/OCD/soft-OCD: auto-recover when BQ flags clear and |I| is low. */
  if (s_current_latched && PowerRails_CurrentClearOk())
  {
    s_current_latched = false;
    s_soft_warn_count = 0U;
    s_soft_fault_count = 0U;
    s_current_reason = PWR_REASON_NONE;
  }

  if (s_pwr_rails_status.bq_valid)
  {
    if (s_pwr_rails_status.scd)
    {
      next = PWR_STATE_FAULT;
      reason = PWR_REASON_SCD;
      s_current_latched = true;
    }
    else if (s_pwr_rails_status.ocd)
    {
      next = PWR_STATE_FAULT;
      reason = PWR_REASON_OCD;
      s_current_latched = true;
    }
    else if (s_pwr_rails_status.occ)
    {
      /* OCC follows BQ flag — no latch; clears when OCC bit drops. */
      next = PWR_STATE_FAULT;
      reason = PWR_REASON_OCC;
    }
  }

  if ((next != PWR_STATE_FAULT) || (reason == PWR_REASON_OCC))
  {
    if ((meas != NULL) && meas->valid && (i_ma < 0))
    {
      if (i_abs >= PROTECT_SOFT_OCD_FAULT_MA)
      {
        if (s_soft_fault_count < 0xFFU)
        {
          s_soft_fault_count++;
        }
      }
      else
      {
        s_soft_fault_count = 0U;
      }

      if (i_abs >= PROTECT_SOFT_OCD_WARN_MA)
      {
        if (s_soft_warn_count < 0xFFU)
        {
          s_soft_warn_count++;
        }
      }
      else
      {
        s_soft_warn_count = 0U;
      }

      if (s_soft_fault_count >= PROTECT_SOFT_OCD_FAULT_DEBOUNCE)
      {
        next = PWR_STATE_FAULT;
        reason = PWR_REASON_SOFT_OCD;
        s_current_latched = true;
      }
      else if ((s_soft_warn_count >= PROTECT_SOFT_OCD_WARN_DEBOUNCE) &&
               (next < PWR_STATE_WARN))
      {
        next = PWR_STATE_WARN;
        reason = PWR_REASON_SOFT_OCD;
      }
    }
    else
    {
      s_soft_warn_count = 0U;
      s_soft_fault_count = 0U;
    }
  }

  /* Hold latched SCD/OCD/soft-OCD until auto-recover clears the latch. */
  if (s_current_latched)
  {
    next = PWR_STATE_FAULT;
    if ((reason != PWR_REASON_SCD) && (reason != PWR_REASON_OCD) &&
        (reason != PWR_REASON_SOFT_OCD))
    {
      reason = s_current_reason;
    }
  }

  s_current_state = next;
  s_current_reason = reason;
}

/* -------------------------------------------------------------------------- */
/* Combine + actuate                                                          */
/* -------------------------------------------------------------------------- */

static uint8_t PowerRails_FanDutyFromThermal(void)
{
  switch (s_thermal_state)
  {
    case PWR_STATE_WARN:
      return THERMAL_FAN_DUTY_WARN;

    case PWR_STATE_LIMIT:
      if (s_thermal_reason == PWR_REASON_COLD_CHARGE)
      {
        return THERMAL_FAN_DUTY_NORMAL;
      }
      return THERMAL_FAN_DUTY_LIMIT;

    case PWR_STATE_FAULT:
      return THERMAL_FAN_DUTY_FAULT;

    case PWR_STATE_NORMAL:
    default:
      return THERMAL_FAN_DUTY_NORMAL;
  }
}

/** 仅真实充电会话：停充 / LIN 断 / 无电流 → 关充电扇；高温另走热保护扇速。 */
static bool PowerRails_WantChargeFan(void)
{
  const charge_status_t *chg = ChargeManager_GetStatus();
  const lin_charger_status_t *lin = LinCharger_GetStatus();
  int16_t i_ma;

  if (ChargeManager_GetState() != CHARGE_STATE_CHARGING)
  {
    return false;
  }

  if ((chg == NULL) || chg->charge_paused)
  {
    return false;
  }

  if (LinCharger_IsCommLost() || ChargePath_IsLinCommInhibit())
  {
    return false;
  }

  if ((lin == NULL) || (lin->session < LIN_SESSION_ACTIVE))
  {
    return false;
  }

  i_ma = chg->pack_current_ma;
  if (i_ma < 0)
  {
    i_ma = (int16_t)(-i_ma);
  }

  return (i_ma >= CHARGE_FAN_MIN_CURRENT_MA);
}

static void PowerRails_ApplyActuators(void)
{
  uint8_t thermal_mask = PWR_MASK_ALL;
  uint8_t protect_mask = PWR_MASK_ALL;
  uint8_t duty = THERMAL_FAN_DUTY_NORMAL;
  bool thermal_chg = false;
  bool thermal_dsg = false;
  bool protect_chg = false;
  bool protect_dsg = false;

  /* Thermal rail / FET (fan duty computed below). */
  switch (s_thermal_state)
  {
    case PWR_STATE_WARN:
      break;
    case PWR_STATE_LIMIT:
      if (s_thermal_reason == PWR_REASON_COLD_CHARGE)
      {
        thermal_mask = PWR_MASK_ALL;
      }
      else
      {
        thermal_mask = PWR_RAILS_DROP_19V;
      }
      thermal_chg = true;
      break;
    case PWR_STATE_FAULT:
      thermal_mask = PWR_RAILS_FAULT_FAN_ENABLE;
      thermal_chg = true;
      thermal_dsg = true;
      break;
    case PWR_STATE_NORMAL:
    default:
      break;
  }

  duty = PowerRails_FanDutyFromThermal();

#if FAN_FORCE_FULL_SPEED
  duty = 100U;
#else
  /* SCRUM-113: 充电主动散热；与热保护取较大值，高温停充后仍可转直到降温。 */
  if (PowerRails_WantChargeFan() &&
      ((s_thermal_state != PWR_STATE_LIMIT) ||
       (s_thermal_reason != PWR_REASON_COLD_CHARGE)))
  {
    if (duty < CHARGE_ACTIVE_FAN_DUTY)
    {
      duty = CHARGE_ACTIVE_FAN_DUTY;
    }
  }
#endif

  /* Current protect rail / FET */
  switch (s_current_state)
  {
    case PWR_STATE_WARN:
      protect_mask = PWR_RAILS_DROP_19V;
      break;
    case PWR_STATE_FAULT:
      if (s_current_reason == PWR_REASON_OCC)
      {
        protect_chg = true;
        protect_mask = PWR_MASK_ALL;
      }
      else
      {
        protect_mask = 0U;
        protect_chg = true;
        protect_dsg = true;
      }
      break;
    default:
      break;
  }

  /* Combined public state: FAULT > LIMIT > WARN > NORMAL */
  if ((s_thermal_state == PWR_STATE_FAULT) ||
      (s_current_state == PWR_STATE_FAULT))
  {
    s_pwr_rails_status.state = PWR_STATE_FAULT;
    if (s_current_state == PWR_STATE_FAULT)
    {
      s_pwr_rails_status.reason = s_current_reason;
    }
    else
    {
      s_pwr_rails_status.reason = s_thermal_reason;
    }
  }
  else if (s_thermal_state == PWR_STATE_LIMIT)
  {
    s_pwr_rails_status.state = PWR_STATE_LIMIT;
    s_pwr_rails_status.reason = s_thermal_reason;
  }
  else if ((s_thermal_state == PWR_STATE_WARN) ||
           (s_current_state == PWR_STATE_WARN))
  {
    s_pwr_rails_status.state = PWR_STATE_WARN;
    s_pwr_rails_status.reason = (s_current_state == PWR_STATE_WARN) ?
                      s_current_reason : s_thermal_reason;
  }
  else
  {
    s_pwr_rails_status.state = PWR_STATE_NORMAL;
    s_pwr_rails_status.reason = PWR_REASON_NONE;
  }

  s_pwr_rails_status.latched = s_thermal_latched || s_current_latched;
  s_pwr_rails_status.fan_duty_percent = duty;
  s_pwr_rails_status.charge_inhibit = thermal_chg || protect_chg;
  s_pwr_rails_status.discharge_inhibit = thermal_dsg || protect_dsg;

  BSP_Fan_SetDutyPercent(duty);
  PowerRails_SetRequest(PWR_REQ_THERMAL, thermal_mask);
  PowerRails_SetRequest(PWR_REQ_PROTECT, protect_mask);
  PowerRails_ApplyRequests();

  ChargePath_SetThermalInhibit(thermal_chg, thermal_dsg);
  ChargePath_SetProtectInhibit(protect_chg, protect_dsg);
  ChargePath_Apply();
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void BSP_PowerRails_Init(void)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    s_pwr_rails_status.rail_on[i] = false;
  }

  s_pwr_rails_status.enabled_mask = 0U;
  s_pwr_rails_status.power_rails_mask = PWR_MASK_ALL;
  s_pwr_rails_status.fan_duty_percent = FAN_FORCE_FULL_SPEED ? 100U : 0U;
  s_pwr_rails_status.charge_inhibit = false;
  s_pwr_rails_status.discharge_inhibit = false;
  s_pwr_rails_status.state = PWR_STATE_NORMAL;
  s_pwr_rails_status.reason = PWR_REASON_NONE;
  s_pwr_rails_status.latched = false;
  s_pwr_rails_status.tmax_c_x10 = 0;
  s_pwr_rails_status.tmin_c_x10 = 0;
  s_pwr_rails_status.die_c_x10 = 0;
  s_pwr_rails_status.sensor_ok = false;
  s_pwr_rails_status.status_a = 0U;
  s_pwr_rails_status.status_b = 0U;
  s_pwr_rails_status.status_c = 0U;
  s_pwr_rails_status.scd = false;
  s_pwr_rails_status.ocd = false;
  s_pwr_rails_status.occ = false;
  s_pwr_rails_status.bq_any = false;
  s_pwr_rails_status.bq_valid = false;
  s_pwr_rails_status.pack_current_ma = 0;

  for (i = 0U; i < (uint8_t)PWR_REQ_COUNT; i++)
  {
    s_request_mask[i] = PWR_MASK_ALL;
  }

  s_thermal_latched = false;
  s_current_latched = false;
  s_thermal_state = PWR_STATE_NORMAL;
  s_thermal_reason = PWR_REASON_NONE;
  s_current_state = PWR_STATE_NORMAL;
  s_current_reason = PWR_REASON_NONE;
  s_soft_warn_count = 0U;
  s_soft_fault_count = 0U;

  BSP_Fan_Init();
  ChargePath_SetThermalInhibit(false, false);
  ChargePath_SetProtectInhibit(false, false);

  if (s_boot_complete)
  {
    PowerRails_ApplyActuators();
  }
}

void BSP_PowerRails_PreBoot(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;

  gpio.Pin = PWR_24V_BYPASS_EN_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_24V_BYPASS_EN_GPIO_Port, &gpio);

  s_boot_complete = false;

  for (i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    s_pwr_rails_status.rail_on[i] = false;
  }

  s_pwr_rails_status.enabled_mask = 0U;
  s_pwr_rails_status.power_rails_mask = PWR_MASK_ALL;

  for (i = 0U; i < (uint8_t)PWR_REQ_COUNT; i++)
  {
    s_request_mask[i] = PWR_MASK_ALL;
  }

  PowerRails_DriveMask(0U);
  PowerRails_Hold24VOff();
}
/*打开电源轨*/
bool BSP_PowerRails_EnableRail(pwr_rail_id_t rail, bool on)
{
  if (rail >= PWR_RAIL_COUNT)
  {
    return false;
  }

  if (rail == PWR_RAIL_5V)
  {
    s_pwr_rails_status.rail_on[PWR_RAIL_5V] =
        on && s_pwr_rails_status.rail_on[PWR_RAIL_6V5];
    return true;
  }

  PowerRails_WriteGpio(rail, on);
  s_pwr_rails_status.rail_on[rail] = on;

  if ((rail == PWR_RAIL_19V) && on)
  {
    if (HAL_GPIO_ReadPin(PWR_19V_EN_GPIO_Port, PWR_19V_EN_Pin) != GPIO_PIN_SET)
    {
      s_pwr_rails_status.rail_on[PWR_RAIL_19V] = false;
      return false;
    }
  }

  if (rail == PWR_RAIL_6V5)
  {
    s_pwr_rails_status.rail_on[PWR_RAIL_5V] = on;
  }

  s_pwr_rails_status.enabled_mask = 0U;
  for (uint8_t i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    if (s_pwr_rails_status.rail_on[i])
    {
      s_pwr_rails_status.enabled_mask =
          (uint8_t)(s_pwr_rails_status.enabled_mask | (1u << i));
    }
  }

  return true;
}

static bool PowerRails_Is19vEnHigh(void)
{
  return (HAL_GPIO_ReadPin(PWR_19V_EN_GPIO_Port, PWR_19V_EN_Pin) == GPIO_PIN_SET);
}

static bool PowerRails_Is19vShorted(void)
{
  if (s_pwr_rails_status.scd)
  {
    return true;
  }

  if (!BSP_AdcRails_IsReady())
  {
    return false;
  }

  BSP_AdcRails_Update();
  return (BSP_AdcRails_GetRailMa(PWR_RAIL_19V) >= PWR_RAIL_19V_SHORT_MA);
}

/* 19V：MCU 已把 EN 拉高，且硬件未反馈短路，延时后认为已打开。 */
static bool PowerRails_Wait19vOn(uint32_t timeout_ms)
{
  if (!s_pwr_rails_status.rail_on[PWR_RAIL_19V] || !PowerRails_Is19vEnHigh())
  {
    return false;
  }

  if (timeout_ms < PWR_RAIL_19V_ON_DELAY_MS)
  {
    return false;
  }

  osDelay(PWR_RAIL_19V_ON_DELAY_MS);

  if (!PowerRails_Is19vEnHigh())
  {
    return false;
  }

  return !PowerRails_Is19vShorted();
}

bool BSP_PowerRails_WaitRailGood(pwr_rail_id_t rail, uint32_t timeout_ms)
{
  const uint32_t poll_ms = 10U;
  const uint8_t confirm_needed = 3U;
  uint32_t elapsed = 0U;
  uint8_t confirm = 0U;

  if ((rail >= PWR_RAIL_COUNT) || !s_pwr_rails_status.rail_on[rail])
  {
    return false;
  }

  /* 19V：EN 拉高且无短路，延时后认为已打开。 */
  if (rail == PWR_RAIL_19V)
  {
    return PowerRails_Wait19vOn(timeout_ms);
  }
  if (!BSP_AdcRails_IsReady())
  {
    return false;
  }

  while (elapsed < timeout_ms)
  {
    if (BSP_AdcRails_IsRailGood(rail))
    {
      confirm++;
      if (confirm >= confirm_needed)
      {
        return true;
      }
    }
    else
    {
      confirm = 0U;
    }

    osDelay(poll_ms);
    elapsed += poll_ms;
  }

  return false;
}

void BSP_PowerRails_SetBootComplete(bool complete)
{
  s_boot_complete = complete;
}

bool BSP_PowerRails_IsBootComplete(void)
{
  return s_boot_complete;
}

void BSP_PowerRails_BootSequence(void)
{
  uint8_t i;

  PowerRails_DriveMask(0U);
  PowerRails_Hold24VOff();

  PowerRails_WriteGpio(PWR_RAIL_12V, true);
  s_pwr_rails_status.rail_on[PWR_RAIL_12V] = true;
  HAL_Delay(300);

  PowerRails_WriteGpio(PWR_RAIL_6V5, true);
  s_pwr_rails_status.rail_on[PWR_RAIL_6V5] = true;
  s_pwr_rails_status.rail_on[PWR_RAIL_5V] = true;
  HAL_Delay(200);

  PowerRails_WriteGpio(PWR_RAIL_19V, true);
  s_pwr_rails_status.rail_on[PWR_RAIL_19V] = true;

  s_pwr_rails_status.enabled_mask = PWR_MASK_ALL;
  s_pwr_rails_status.power_rails_mask = PWR_MASK_ALL;
  for (i = 0U; i < (uint8_t)PWR_REQ_COUNT; i++)
  {
    s_request_mask[i] = PWR_MASK_ALL;
  }
}

void BSP_PowerRails_UpdateBqSafety(uint8_t status_a, uint8_t status_b,
                                   uint8_t status_c, bool valid)
{
  s_pwr_rails_status.status_a = status_a;
  s_pwr_rails_status.status_b = status_b;
  s_pwr_rails_status.status_c = status_c;
  s_pwr_rails_status.bq_valid = valid;

  if (!valid)
  {
    s_pwr_rails_status.scd = false;
    s_pwr_rails_status.ocd = false;
    s_pwr_rails_status.occ = false;
    s_pwr_rails_status.bq_any = false;
    return;
  }

  s_pwr_rails_status.scd = ((status_a & BQ76942_SA_SCD) != 0U);
  s_pwr_rails_status.ocd = ((status_a & (BQ76942_SA_OCD1 | BQ76942_SA_OCD2)) != 0U);
  s_pwr_rails_status.occ = ((status_a & BQ76942_SA_OCC) != 0U);
  s_pwr_rails_status.bq_any = ((status_a != 0U) || (status_b != 0U) || (status_c != 0U));
}

void BSP_PowerRails_Process(void)
{
  if (!s_boot_complete)
  {
    return;
  }

  PowerRails_EvalThermal();
  PowerRails_EvalCurrent();
  PowerRails_ApplyActuators();
}

const pwr_rails_status_t *BSP_PowerRails_GetStatus(void)
{
  return &s_pwr_rails_status;
}

pwr_state_t BSP_PowerRails_GetState(void)
{
  return s_pwr_rails_status.state;
}

bool BSP_PowerRails_ClearFault(void)
{
  bool had_fault = (s_pwr_rails_status.state == PWR_STATE_FAULT) ||
                   s_thermal_latched || s_current_latched;

  if (!had_fault)
  {
    return false;
  }

  /* Same auto-recover rules as Process(); re-evaluate immediately. */
  PowerRails_EvalThermal();
  PowerRails_EvalCurrent();
  PowerRails_ApplyActuators();

  return (s_pwr_rails_status.state != PWR_STATE_FAULT) &&
         (!s_thermal_latched) && (!s_current_latched);
}
