/**
 ******************************************************************************
 * @file    cell_balance_manager.c
 * @brief   6S NMC passive balancing: top fine-balance + mid highest-cell protect.
 ******************************************************************************
 */
#include "cell_balance_manager.h"
#include "bq76942.h"
#include "bsp_power_rails.h"
#include "charge_path.h"
#include "app_freertos.h"

static balance_status_t s_status;
static bool s_charger_present_explicit;
static bool s_charger_present_value;
static bool s_mid_session_active;

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

static bool Balance_IsMidState(balance_state_t state)
{
  return (state == BALANCE_STATE_MID_RELAX) ||
         (state == BALANCE_STATE_MID_PROTECT);
}

static void Balance_StopAll(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != NULL)
  {
    (void)BQ76942_SetBalanceMask(hi2c, 0U);
  }

  s_status.active_mask = 0U;
}

static void Balance_ApplyChargeInhibit(bool inhibit)
{
  s_status.imbalance_charge_inhibit = inhibit;
  ChargePath_SetImbalanceChargeInhibit(inhibit);
  ChargePath_Apply();
}

static void Balance_ResetMidSession(void)
{
  s_status.mid_class = BALANCE_MID_CLASS_NONE;
  s_status.mid_observe_done = false;
  s_status.mid_delta_at_pause_mv = 0U;
  s_status.mid_vmax_at_pause_mv = 0U;
  s_status.mid_relax_count = 0U;
  s_status.mid_protect_count = 0U;
  s_status.mid_protect_cooldown = 0U;
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

/* Bleed only the highest cell(s) still in the warn/danger band. */
static uint16_t Balance_SelectProtectMask(const uint16_t *cell_mv,
                                          uint16_t vmax_mv,
                                          uint8_t count)
{
  uint16_t mask = 0U;
  uint8_t pick;
  uint16_t floor_mv;

  if (vmax_mv > BALANCE_MID_PROTECT_NEAR_MV)
  {
    floor_mv = (uint16_t)(vmax_mv - BALANCE_MID_PROTECT_NEAR_MV);
  }
  else
  {
    floor_mv = 0U;
  }

  if (floor_mv < BALANCE_MID_VMAX_WARN_MV)
  {
    floor_mv = BALANCE_MID_VMAX_WARN_MV;
  }

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

      if (cell_mv[i] < floor_mv)
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

static bool Balance_EvalCriticalFault(const pwr_rails_status_t *protect,
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

  if ((protect != NULL) &&
      ((protect->state >= PWR_STATE_LIMIT) || (!protect->sensor_ok)))
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

static void Balance_UpdateTopChargeInhibit(bool sample_stable)
{
  bool inhibit = ChargePath_IsImbalanceChargeInhibit();

  if (!sample_stable)
  {
    s_status.imbalance_charge_inhibit = inhibit;
    ChargePath_Apply();
    return;
  }

  if (inhibit)
  {
    if (s_status.delta_mv <= CHARGE_IMBALANCE_RESUME_DELTA_MV)
    {
      inhibit = false;
    }
  }
  else
  {
    if ((s_status.delta_mv >= CHARGE_IMBALANCE_STOP_DELTA_MV) &&
        (s_status.chg_fet_on || s_status.charger_present))
    {
      inhibit = true;
    }
  }

  Balance_ApplyChargeInhibit(inhibit);
}

static void Balance_UpdateChargerPath(void)
{
  bool path_ok;
  bool raw;
  const bool imbalance_hold = ChargePath_IsImbalanceChargeInhibit();

  if (s_charger_present_explicit)
  {
    s_status.charger_present = s_charger_present_value;
  }
  else
  {
    /*
     * Infer present from CHG FET, or keep present while imbalance paused
     * charging (CFETOFF forced off so FET status alone would falsely clear).
     */
    s_status.charger_present = s_status.chg_fet_on || imbalance_hold;
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

  /*
   * Balance / mid-protect may continue while imbalance holds charge off:
   * charger_present && (chg_fet_on || imbalance_hold) && !discharge.
   */
  path_ok = s_status.charger_present &&
            (s_status.chg_fet_on || imbalance_hold) &&
            (!s_status.discharge_detected);

  if (!path_ok)
  {
    s_status.charger_active = false;
    s_status.charge_stable_count = 0U;
    s_status.charger_active_stable = false;
    return;
  }

  raw = path_ok;
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

static void Balance_EnterMidRelax(void)
{
  s_status.mid_delta_at_pause_mv = s_status.delta_mv;
  s_status.mid_vmax_at_pause_mv = s_status.vmax_mv;
  s_status.mid_relax_count = 0U;
  s_status.mid_protect_count = 0U;
  s_status.state = BALANCE_STATE_MID_RELAX;
  s_status.inhibit_reason = BALANCE_INHIBIT_MID_OBSERVE;
  Balance_ApplyChargeInhibit(true);
  Balance_UpdateChargerPath();
}

static void Balance_LeaveMidToIdle(I2C_HandleTypeDef *hi2c,
                                   balance_inhibit_reason_t reason)
{
  Balance_StopAll(hi2c);
  s_status.mid_relax_count = 0U;
  s_status.mid_protect_count = 0U;
  s_status.state = BALANCE_STATE_IDLE;
  s_status.inhibit_reason = reason;
  Balance_ApplyChargeInhibit(false);
  Balance_UpdateChargerPath();
}

static void Balance_ClassifyAfterRelax(void)
{
  const uint16_t rest_delta = s_status.delta_mv;
  const uint16_t rest_vmax = s_status.vmax_mv;
  uint16_t shrink = 0U;

  if (s_status.mid_delta_at_pause_mv > rest_delta)
  {
    shrink = (uint16_t)(s_status.mid_delta_at_pause_mv - rest_delta);
  }

  /* After IR falls, highest cell still at the ceiling → real and threatening. */
  if (rest_vmax >= BALANCE_MID_VMAX_DANGER_MV)
  {
    s_status.mid_class = BALANCE_MID_CLASS_REAL_THREAT;
    s_status.mid_observe_done = true;
    return;
  }

  s_status.mid_observe_done = true;

  if ((shrink >= BALANCE_MID_FAKE_SHRINK_MV) ||
      (rest_delta < BALANCE_MID_REST_REAL_DELTA_MV))
  {
    s_status.mid_class = BALANCE_MID_CLASS_FAKE_IR;
    return;
  }

  s_status.mid_class = BALANCE_MID_CLASS_REAL_SAFE;
}

static bool Balance_EnterMidProtect(I2C_HandleTypeDef *hi2c)
{
  const uint16_t mask = Balance_SelectProtectMask(s_status.cell_mv,
                                                  s_status.vmax_mv,
                                                  BMS_CELL_COUNT);

  s_status.mid_class = BALANCE_MID_CLASS_REAL_THREAT;
  s_status.mid_observe_done = true;
  s_status.mid_protect_count = 0U;
  s_status.state = BALANCE_STATE_MID_PROTECT;
  s_status.inhibit_reason = BALANCE_INHIBIT_MID_PROTECT;
  Balance_ApplyChargeInhibit(true);
  Balance_UpdateChargerPath();

  if (!Balance_ApplyMask(hi2c, mask))
  {
    return false;
  }

  return true;
}

static bool Balance_MidNeedRelax(void)
{
  const bool charge_ctx = s_status.charger_present &&
                          (s_status.chg_fet_on ||
                           ChargePath_IsImbalanceChargeInhibit());

  if (!charge_ctx || s_status.discharge_detected)
  {
    return false;
  }

  if ((s_status.vmax_mv >= BALANCE_MID_VMAX_DANGER_MV) &&
      (s_status.mid_protect_cooldown == 0U))
  {
    return true;
  }

  if ((!s_status.mid_observe_done) &&
      s_status.charger_active_stable &&
      (s_status.delta_mv >= BALANCE_MID_OBSERVE_DELTA_MV))
  {
    return true;
  }

  return false;
}

static void Balance_TickMidCooldown(void)
{
  if ((s_status.state != BALANCE_STATE_MID_PROTECT) &&
      (s_status.mid_protect_cooldown > 0U))
  {
    s_status.mid_protect_cooldown--;
  }
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
  s_status.imbalance_charge_inhibit = false;
  s_charger_present_explicit = false;
  s_charger_present_value = false;
  s_mid_session_active = false;
  Balance_ResetMidSession();
  ChargePath_SetImbalanceChargeInhibit(false);
}

void Balance_SetEnabled(bool enable)
{
  s_status.user_enabled = enable;

  if (!enable)
  {
    s_status.state = BALANCE_STATE_DISABLED;
    s_status.inhibit_reason = BALANCE_INHIBIT_DISABLED;
    Balance_ResetMidSession();
    Balance_ApplyChargeInhibit(false);
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
  const pwr_rails_status_t *thermal = BSP_PowerRails_GetStatus();
  uint32_t comm_fail = Bms_GetBqCommFailCount();
  bool sample_ok;
  bool temperature_normal;
  bool critical_fault;
  bool bq_protect = false;
  bool common_ok;
  bool sample_stable;
  bool in_top_window;
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
      Balance_ResetMidSession();
      Balance_ApplyChargeInhibit(false);
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

  if (s_status.charger_present)
  {
    s_mid_session_active = true;
  }
  else if (s_mid_session_active && !ChargePath_IsImbalanceChargeInhibit())
  {
    Balance_ResetMidSession();
    s_mid_session_active = false;
  }

  temperature_normal = Balance_EvalTemperature();

  s_status.top_start_ready = Balance_IsTopReady(
      s_status.soc_valid, s_status.soc_percent, s_status.vmin_mv,
      BALANCE_SOC_START_PERCENT, BALANCE_VMIN_START_MV);

  s_status.top_hold_ready = Balance_IsTopReady(
      s_status.soc_valid, s_status.soc_percent, s_status.vmin_mv,
      BALANCE_SOC_HOLD_PERCENT, BALANCE_VMIN_HOLD_MV);

  sample_stable = sample_ok &&
                  (s_status.stable_count >= BALANCE_SAMPLE_STABLE_COUNT);

  critical_fault = false;
  if (comm_fail >= BALANCE_COMM_FAIL_THRESHOLD)
  {
    critical_fault = true;
    s_status.inhibit_reason = BALANCE_INHIBIT_COMM;
  }
  else if (sample_stable)
  {
    critical_fault = Balance_EvalCriticalFault(thermal, bq_protect, comm_fail);
  }

  if (critical_fault)
  {
    Balance_StopAll(hi2c);
    if (Balance_IsMidState(s_status.state))
    {
      Balance_ApplyChargeInhibit(false);
    }
    s_status.state = BALANCE_STATE_INHIBITED;
    return;
  }

  if (!sample_ok || (s_status.stable_count < BALANCE_SAMPLE_STABLE_COUNT))
  {
    if ((s_status.state == BALANCE_STATE_ACTIVE) ||
        (s_status.state == BALANCE_STATE_MID_PROTECT))
    {
      Balance_StopAll(hi2c);
      if (s_status.state == BALANCE_STATE_ACTIVE)
      {
        s_status.state = BALANCE_STATE_IDLE;
      }
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

    if ((s_status.state == BALANCE_STATE_ACTIVE) ||
        Balance_IsMidState(s_status.state))
    {
      Balance_StopAll(hi2c);
      if (Balance_IsMidState(s_status.state))
      {
        Balance_ApplyChargeInhibit(false);
      }
      s_status.state = BALANCE_STATE_IDLE;
    }

    return;
  }

  Balance_TickMidCooldown();

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

  /* Hold window is only for staying in ACTIVE; mid protect still runs
   * until the start window (SOC 90% / vmin 3900) is actually reached. */
  in_top_window = s_status.top_start_ready ||
                  (s_status.state == BALANCE_STATE_ACTIVE);

  /* Charger gone: drop mid immediately (explicit present is authoritative). */
  if (!s_status.charger_present && Balance_IsMidState(s_status.state))
  {
    Balance_LeaveMidToIdle(hi2c, BALANCE_INHIBIT_NOT_CHARGING);
    Balance_ResetMidSession();
    s_mid_session_active = false;
    return;
  }

  if (s_status.discharge_detected && Balance_IsMidState(s_status.state))
  {
    Balance_LeaveMidToIdle(hi2c, BALANCE_INHIBIT_DISCHARGE);
    return;
  }

  /* Mid-relax: pause only, no bleed. Wait for IR to fall, then classify. */
  if (s_status.state == BALANCE_STATE_MID_RELAX)
  {
    if (s_status.top_start_ready && s_status.start_conditions_ok)
    {
      uint16_t mask;

      Balance_ApplyChargeInhibit(false);
      Balance_UpdateTopChargeInhibit(sample_stable);
      Balance_UpdateChargerPath();
      mask = Balance_SelectMask(s_status.cell_mv, s_status.vmin_mv,
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
      return;
    }

    if (s_status.mid_relax_count < 255U)
    {
      s_status.mid_relax_count++;
    }

    if (s_status.mid_relax_count < BALANCE_MID_RELAX_COUNT)
    {
      s_status.inhibit_reason = BALANCE_INHIBIT_MID_OBSERVE;
      return;
    }

    Balance_ClassifyAfterRelax();

    if (s_status.mid_class == BALANCE_MID_CLASS_REAL_THREAT)
    {
      if (!Balance_EnterMidProtect(hi2c))
      {
        Balance_StopAll(hi2c);
        Balance_ApplyChargeInhibit(false);
        s_status.state = BALANCE_STATE_INHIBITED;
        s_status.inhibit_reason = BALANCE_INHIBIT_COMM;
      }
      return;
    }

    Balance_LeaveMidToIdle(hi2c, s_status.top_start_ready ?
                           BALANCE_INHIBIT_DELTA_LOW :
                           BALANCE_INHIBIT_TOP_NOT_READY);
    return;
  }

  /* Mid-protect: short bleed of the highest cell until vmax leaves danger. */
  if (s_status.state == BALANCE_STATE_MID_PROTECT)
  {
    if (s_status.top_start_ready && s_status.start_conditions_ok)
    {
      uint16_t mask;

      Balance_UpdateTopChargeInhibit(sample_stable);
      Balance_UpdateChargerPath();
      mask = Balance_SelectMask(s_status.cell_mv, s_status.vmin_mv,
                                BMS_CELL_COUNT);
      if (Balance_ApplyMask(hi2c, mask))
      {
        s_status.state = BALANCE_STATE_ACTIVE;
        s_status.inhibit_reason = BALANCE_INHIBIT_NONE;
      }
      else
      {
        Balance_StopAll(hi2c);
        s_status.state = BALANCE_STATE_INHIBITED;
        s_status.inhibit_reason = BALANCE_INHIBIT_COMM;
      }
      return;
    }

    if (s_status.mid_protect_count < 255U)
    {
      s_status.mid_protect_count++;
    }

    if ((s_status.vmax_mv <= BALANCE_MID_VMAX_SAFE_MV) ||
        (s_status.mid_protect_count >= BALANCE_MID_PROTECT_MAX_TICKS))
    {
      s_status.mid_protect_cooldown = BALANCE_MID_PROTECT_COOLDOWN;
      Balance_LeaveMidToIdle(hi2c, BALANCE_INHIBIT_NONE);
      return;
    }

    {
      uint16_t mask = Balance_SelectProtectMask(s_status.cell_mv,
                                                s_status.vmax_mv,
                                                BMS_CELL_COUNT);
      (void)Balance_ApplyMask(hi2c, mask);
    }

    s_status.inhibit_reason = BALANCE_INHIBIT_MID_PROTECT;
    return;
  }

  if (s_status.state == BALANCE_STATE_ACTIVE)
  {
    /* Allow ACTIVE while imbalance holds charge off (chg_fet may read off). */
    if (!s_status.charger_present && !ChargePath_IsImbalanceChargeInhibit())
    {
      Balance_StopAll(hi2c);
      s_status.state = BALANCE_STATE_IDLE;
      s_status.inhibit_reason = BALANCE_INHIBIT_NOT_CHARGING;
      return;
    }

    if (!s_status.chg_fet_on && !ChargePath_IsImbalanceChargeInhibit())
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

    Balance_UpdateTopChargeInhibit(sample_stable);
    Balance_UpdateChargerPath();

    {
      uint16_t mask = Balance_SelectMask(s_status.cell_mv, s_status.vmin_mv,
                                         BMS_CELL_COUNT);
      (void)Balance_ApplyMask(hi2c, mask);
    }

    s_status.state = BALANCE_STATE_ACTIVE;
    s_status.inhibit_reason = BALANCE_INHIBIT_NONE;
    return;
  }

  /* Not ACTIVE / mid: evaluate start path only. */
  Balance_StopAll(hi2c);

  if (in_top_window)
  {
    Balance_UpdateTopChargeInhibit(sample_stable);
    Balance_UpdateChargerPath();
  }
  else if (ChargePath_IsImbalanceChargeInhibit())
  {
    /* Mid-charge must not hold a leftover top Δ pause. */
    Balance_ApplyChargeInhibit(false);
    Balance_UpdateChargerPath();
  }

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
    return;
  }

  if ((!in_top_window) && Balance_MidNeedRelax())
  {
    Balance_EnterMidRelax();
    return;
  }

  if (!s_status.charger_active_stable)
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
