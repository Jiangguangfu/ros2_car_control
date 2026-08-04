/**
 ******************************************************************************
 * @file    cell_balance_manager.c
 * @brief   6S NMC passive balancing policy for BQ76942.
 ******************************************************************************
 */
#include "cell_balance_manager.h"
#include "bq76942.h"
#include "thermal_manager.h"
#include "app_freertos.h"

static balance_status_t s_status;
static bool s_charger_present_explicit;
static bool s_charger_present_value;

static bool Balance_IsTopReady(bool soc_valid,
                               uint8_t soc_percent,
                               uint16_t vmin_mv,
                               uint8_t soc_threshold,
                               uint16_t vmin_threshold)
{
  if (soc_valid)
  {
    return (soc_percent >= soc_threshold);
  }

  return (vmin_mv >= vmin_threshold);
}

static void Balance_StopAll(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != NULL)
  {
    (void)BQ76942_SetBalanceMask(hi2c, 0U);
  }

  s_status.active_mask = 0U;
}

static bool Balance_CellsSampleValid(const uint16_t *cell_mv, uint8_t count)
{
  uint8_t i;

  for (i = 0U; i < count; i++)
  {
    if ((cell_mv[i] < BALANCE_CELL_MV_MIN_VALID) ||
        (cell_mv[i] > BALANCE_CELL_MV_MAX_VALID))
    {
      return false;
    }
  }

  return true;
}

static void Balance_UpdateMinMax(const uint16_t *cell_mv, uint8_t count)
{
  uint8_t i;
  uint16_t vmin = cell_mv[0U];
  uint16_t vmax = cell_mv[0U];

  for (i = 1U; i < count; i++)
  {
    if (cell_mv[i] < vmin)
    {
      vmin = cell_mv[i];
    }

    if (cell_mv[i] > vmax)
    {
      vmax = cell_mv[i];
    }
  }

  s_status.vmin_mv = vmin;
  s_status.vmax_mv = vmax;
  s_status.delta_mv = (uint16_t)(vmax - vmin);
}

static uint16_t Balance_SelectMask(const uint16_t *cell_mv, uint16_t vmin_mv,
                                   uint8_t count)
{
  uint16_t mask = 0U;
  uint8_t pick;

  for (pick = 0U; pick < BALANCE_MAX_CELLS_AT_ONCE; pick++)
  {
    uint8_t i;
    int8_t best_i = -1;
    uint16_t best_mv = 0U;

    for (i = 0U; i < count; i++)
    {
      if ((mask & (1U << i)) != 0U)
      {
        continue;
      }

      if (cell_mv[i] < (uint16_t)(vmin_mv + BALANCE_STOP_DELTA_MV))
      {
        continue;
      }

      if ((best_i < 0) || (cell_mv[i] > best_mv))
      {
        best_i = (int8_t)i;
        best_mv = cell_mv[i];
      }
    }

    if (best_i < 0)
    {
      break;
    }

    mask |= (1U << (uint8_t)best_i);
  }

  return (uint16_t)(mask & BMS_BALANCE_MASK_VALID);
}

static bool Balance_ApplyMask(I2C_HandleTypeDef *hi2c, uint16_t mask)
{
  uint16_t readback = 0U;

  mask &= BMS_BALANCE_MASK_VALID;

  if (!BQ76942_SetBalanceMask(hi2c, mask))
  {
    return false;
  }

  if (BQ76942_ReadBalanceMask(hi2c, &readback))
  {
    s_status.active_mask = (uint16_t)(readback & BMS_BALANCE_MASK_VALID);
  }
  else
  {
    s_status.active_mask = mask;
  }

  return true;
}

static bool Balance_EvalTemperature(void)
{
  const bq76942_temp_t *temp = Bms_GetBqTemperatures();

  if ((temp == NULL) || (!temp->valid))
  {
    return false;
  }

  s_status.die_c_x10 = temp->int_temp_c_x10;
  s_status.tmax_cell_c_x10 = (temp->ts1_temp_c_x10 > temp->ts2_temp_c_x10) ?
                             temp->ts1_temp_c_x10 : temp->ts2_temp_c_x10;
  s_status.tmin_cell_c_x10 = (temp->ts1_temp_c_x10 < temp->ts2_temp_c_x10) ?
                             temp->ts1_temp_c_x10 : temp->ts2_temp_c_x10;

  if (s_status.tmin_cell_c_x10 < BALANCE_CELL_TEMP_MIN_CX10)
  {
    return false;
  }

  if (s_status.tmax_cell_c_x10 > BALANCE_CELL_TEMP_MAX_CX10)
  {
    return false;
  }

  if (s_status.die_c_x10 > BALANCE_DIE_TEMP_MAX_CX10)
  {
    return false;
  }

  return true;
}

static bool Balance_EvalCriticalFault(const thermal_status_t *thermal,
                                      bool bq_protect,
                                      uint32_t comm_fail_count)
{
  if (s_status.vmin_mv < BALANCE_HARD_MIN_CELL_MV)
  {
    s_status.inhibit_reason = BALANCE_INHIBIT_HARD_UNDERVOLT;
    return true;
  }

  if (s_status.vmax_mv > BALANCE_NORMAL_MAX_CELL_MV)
  {
    s_status.inhibit_reason = BALANCE_INHIBIT_OVERVOLT;
    return true;
  }

  if ((thermal != NULL) &&
      ((thermal->state >= THERMAL_STATE_LIMIT) || (!thermal->sensor_ok)))
  {
    s_status.inhibit_reason = BALANCE_INHIBIT_THERMAL;
    return true;
  }

  if (comm_fail_count >= BALANCE_COMM_FAIL_THRESHOLD)
  {
    s_status.inhibit_reason = BALANCE_INHIBIT_COMM;
    return true;
  }

  if (bq_protect)
  {
    s_status.inhibit_reason = BALANCE_INHIBIT_BQ_PROTECT;
    return true;
  }

  return false;
}

static void Balance_UpdateChargerPath(void)
{
  bool raw;

  if (s_charger_present_explicit)
  {
    s_status.charger_present = s_charger_present_value;
  }
  else
  {
    /* P0: no dedicated charger sense — infer from CHG FET path. */
    s_status.charger_present = s_status.chg_fet_on;
  }

  if (!s_status.charger_present || !s_status.chg_fet_on)
  {
    s_status.charger_active = false;
    s_status.charge_stable_count = 0U;
    return;
  }

  if (s_status.pack_current_ma <= BALANCE_DISCHARGE_EXIT_MA)
  {
    if (s_status.discharge_exit_count < 255U)
    {
      s_status.discharge_exit_count++;
    }

    if (s_status.discharge_exit_count >= BALANCE_DISCHARGE_EXIT_DEBOUNCE)
    {
      s_status.discharge_detected = true;
    }
  }
  else if (s_status.pack_current_ma >= BALANCE_DISCHARGE_RECOVER_MA)
  {
    s_status.discharge_detected = false;
    s_status.discharge_exit_count = 0U;
  }

  raw = s_status.charger_present &&
        s_status.chg_fet_on &&
        (!s_status.discharge_detected);

  s_status.charger_active = raw;

  if (raw)
  {
    if (s_status.charge_stable_count < 255U)
    {
      s_status.charge_stable_count++;
    }
  }
  else
  {
    s_status.charge_stable_count = 0U;
  }

  s_status.charger_active_stable =
      (s_status.charge_stable_count >= BALANCE_CHARGE_DEBOUNCE_COUNT);
}

void Balance_Init(void)
{
  s_status.user_enabled = true;
  s_status.state = BALANCE_STATE_IDLE;
  s_status.inhibit_reason = BALANCE_INHIBIT_NONE;
  s_status.soc_valid = false;
  s_status.soc_percent = 0U;
  s_status.active_mask = 0U;
  s_status.stable_count = 0U;
  s_status.charge_stable_count = 0U;
  s_status.discharge_exit_count = 0U;
  s_status.discharge_detected = false;
  s_charger_present_explicit = false;
  s_charger_present_value = false;
}

void Balance_SetEnabled(bool enable)
{
  s_status.user_enabled = enable;

  if (!enable)
  {
    s_status.state = BALANCE_STATE_DISABLED;
    s_status.inhibit_reason = BALANCE_INHIBIT_DISABLED;
  }
  else if (s_status.state == BALANCE_STATE_DISABLED)
  {
    s_status.state = BALANCE_STATE_IDLE;
    s_status.inhibit_reason = BALANCE_INHIBIT_NONE;
  }
}

bool Balance_IsEnabled(void)
{
  return s_status.user_enabled;
}

void Balance_SetChargerPresent(bool present)
{
  s_charger_present_explicit = true;
  s_charger_present_value = present;
}

void Balance_SetSoc(uint8_t soc_percent, bool valid)
{
  if (soc_percent > 100U)
  {
    soc_percent = 100U;
  }

  s_status.soc_percent = soc_percent;
  s_status.soc_valid = valid;
}

const balance_status_t *Balance_GetStatus(void)
{
  return &s_status;
}

void Balance_Process(I2C_HandleTypeDef *hi2c)
{
  bq76942_cells_t cells;
  const thermal_status_t *thermal = Thermal_GetStatus();
  uint32_t comm_fail = Bms_GetBqCommFailCount();
  bool sample_ok;
  bool temperature_normal;
  bool critical_fault;
  bool bq_protect = false;
  bool common_ok;
  uint8_t i;

  s_status.start_conditions_ok = false;
  s_status.hold_conditions_ok = false;
  s_status.delta_ok = (s_status.delta_mv <= BALANCE_STOP_DELTA_MV);

  if ((hi2c == NULL) || (!s_status.user_enabled))
  {
    if (!s_status.user_enabled)
    {
      Balance_StopAll(hi2c);
      s_status.state = BALANCE_STATE_DISABLED;
      s_status.inhibit_reason = BALANCE_INHIBIT_DISABLED;
    }
    return;
  }

  sample_ok = BQ76942_ReadCellVoltages(hi2c, BMS_CELL_COUNT, &cells);
  if (sample_ok)
  {
    for (i = 0U; i < BMS_CELL_COUNT; i++)
    {
      s_status.cell_mv[i] = cells.cell_mv[i];
    }
    s_status.stack_mv = cells.stack_mv;
    Balance_UpdateMinMax(s_status.cell_mv, BMS_CELL_COUNT);
  }

  if (!BQ76942_ReadPackCurrent(hi2c, &s_status.pack_current_ma))
  {
    sample_ok = false;
  }

  {
    uint8_t fet_status = 0U;

    if (BQ76942_ReadFetStatus(hi2c, &fet_status))
    {
      s_status.chg_fet_on = ((fet_status & BQ76942_FETSTAT_CHG_FET) != 0U);
    }
    else
    {
      sample_ok = false;
    }
  }

  if (!BQ76942_ReadSafetyStatus(hi2c, &bq_protect))
  {
    sample_ok = false;
  }

  {
    const bq76942_temp_t *temp = Bms_GetBqTemperatures();
    bool i2c_ok = sample_ok && Balance_CellsSampleValid(s_status.cell_mv, BMS_CELL_COUNT);

    if ((temp == NULL) || (!temp->valid))
    {
      i2c_ok = false;
    }

    if (i2c_ok)
    {
      if (s_status.stable_count < 255U)
      {
        s_status.stable_count++;
      }
      Bms_RecordBqI2cResult(true);
    }
    else
    {
      s_status.stable_count = 0U;
      Bms_RecordBqI2cResult(false);
    }
  }

  Balance_UpdateChargerPath();

  temperature_normal = Balance_EvalTemperature();

  s_status.top_start_ready = Balance_IsTopReady(
      s_status.soc_valid, s_status.soc_percent, s_status.vmin_mv,
      BALANCE_SOC_START_PERCENT, BALANCE_VMIN_START_MV);

  s_status.top_hold_ready = Balance_IsTopReady(
      s_status.soc_valid, s_status.soc_percent, s_status.vmin_mv,
      BALANCE_SOC_HOLD_PERCENT, BALANCE_VMIN_HOLD_MV);

  critical_fault = false;
  if (comm_fail >= BALANCE_COMM_FAIL_THRESHOLD)
  {
    critical_fault = true;
    s_status.inhibit_reason = BALANCE_INHIBIT_COMM;
  }
  else if (sample_ok && (s_status.stable_count >= BALANCE_SAMPLE_STABLE_COUNT))
  {
    critical_fault = Balance_EvalCriticalFault(thermal, bq_protect, comm_fail);
  }

  if (critical_fault)
  {
    Balance_StopAll(hi2c);
    s_status.state = BALANCE_STATE_INHIBITED;
    return;
  }

  if (!sample_ok || (s_status.stable_count < BALANCE_SAMPLE_STABLE_COUNT))
  {
    if (s_status.state == BALANCE_STATE_ACTIVE)
    {
      Balance_StopAll(hi2c);
      s_status.state = BALANCE_STATE_IDLE;
    }

    s_status.inhibit_reason = sample_ok ? BALANCE_INHIBIT_SAMPLE_UNSTABLE :
                              BALANCE_INHIBIT_COMM;
    return;
  }

  common_ok =
      s_status.user_enabled &&
      (s_status.stable_count >= BALANCE_SAMPLE_STABLE_COUNT) &&
      (s_status.vmin_mv >= BALANCE_HARD_MIN_CELL_MV) &&
      (s_status.vmax_mv <= BALANCE_NORMAL_MAX_CELL_MV) &&
      temperature_normal;

  if (!common_ok)
  {
    if (!temperature_normal)
    {
      s_status.inhibit_reason = BALANCE_INHIBIT_THERMAL;
    }
    else if (s_status.stable_count < BALANCE_SAMPLE_STABLE_COUNT)
    {
      s_status.inhibit_reason = BALANCE_INHIBIT_SAMPLE_UNSTABLE;
    }
    else
    {
      s_status.inhibit_reason = BALANCE_INHIBIT_NONE;
    }

    if (s_status.state == BALANCE_STATE_ACTIVE)
    {
      Balance_StopAll(hi2c);
      s_status.state = BALANCE_STATE_IDLE;
    }

    return;
  }

  s_status.start_conditions_ok =
      common_ok &&
      s_status.charger_active_stable &&
      s_status.top_start_ready &&
      (s_status.delta_mv >= BALANCE_START_DELTA_MV);

  s_status.hold_conditions_ok =
      common_ok &&
      s_status.charger_active &&
      s_status.top_hold_ready &&
      (s_status.delta_mv > BALANCE_STOP_DELTA_MV);

  s_status.delta_ok = (s_status.delta_mv <= BALANCE_STOP_DELTA_MV);

  if (s_status.state == BALANCE_STATE_ACTIVE)
  {
    if (!s_status.charger_present || !s_status.chg_fet_on)
    {
      Balance_StopAll(hi2c);
      s_status.state = BALANCE_STATE_IDLE;
      s_status.inhibit_reason = BALANCE_INHIBIT_NOT_CHARGING;
      return;
    }

    if (s_status.discharge_detected)
    {
      Balance_StopAll(hi2c);
      s_status.state = BALANCE_STATE_IDLE;
      s_status.inhibit_reason = BALANCE_INHIBIT_DISCHARGE;
      return;
    }

    if (!s_status.hold_conditions_ok || !temperature_normal)
    {
      Balance_StopAll(hi2c);
      s_status.state = BALANCE_STATE_IDLE;

      if (!s_status.top_hold_ready)
      {
        s_status.inhibit_reason = BALANCE_INHIBIT_TOP_NOT_READY;
      }
      else if (s_status.delta_mv <= BALANCE_STOP_DELTA_MV)
      {
        s_status.inhibit_reason = BALANCE_INHIBIT_DELTA_LOW;
      }
      else if (!s_status.charger_active)
      {
        s_status.inhibit_reason = BALANCE_INHIBIT_NOT_CHARGING;
      }
      else
      {
        s_status.inhibit_reason = BALANCE_INHIBIT_NONE;
      }

      return;
    }

    {
      uint16_t mask = Balance_SelectMask(s_status.cell_mv, s_status.vmin_mv,
                                         BMS_CELL_COUNT);
      (void)Balance_ApplyMask(hi2c, mask);
    }

    s_status.state = BALANCE_STATE_ACTIVE;
    s_status.inhibit_reason = BALANCE_INHIBIT_NONE;
    return;
  }

  /* Not ACTIVE: evaluate start path only. */
  Balance_StopAll(hi2c);

  if (s_status.start_conditions_ok)
  {
    uint16_t mask = Balance_SelectMask(s_status.cell_mv, s_status.vmin_mv,
                                       BMS_CELL_COUNT);

    if (Balance_ApplyMask(hi2c, mask))
    {
      s_status.state = BALANCE_STATE_ACTIVE;
      s_status.inhibit_reason = BALANCE_INHIBIT_NONE;
    }
    else
    {
      s_status.state = BALANCE_STATE_INHIBITED;
      s_status.inhibit_reason = BALANCE_INHIBIT_COMM;
    }
  }
  else if (!s_status.charger_active_stable)
  {
    s_status.state = BALANCE_STATE_WAIT_CHARGE;
    s_status.inhibit_reason = BALANCE_INHIBIT_NOT_CHARGING;
  }
  else if (!s_status.top_start_ready)
  {
    s_status.state = BALANCE_STATE_IDLE;
    s_status.inhibit_reason = BALANCE_INHIBIT_TOP_NOT_READY;
  }
  else if (s_status.delta_mv < BALANCE_START_DELTA_MV)
  {
    s_status.state = BALANCE_STATE_IDLE;
    s_status.inhibit_reason = BALANCE_INHIBIT_DELTA_LOW;
  }
  else
  {
    s_status.state = BALANCE_STATE_IDLE;
    s_status.inhibit_reason = BALANCE_INHIBIT_NONE;
  }
}
