/**
 ******************************************************************************
 * @file    soh_estimator.c
 * @brief   SOH：永久容量保持率 + 自动标定；一致性/告警分离。
 ******************************************************************************
 */
#include "soh_estimator.h"

#include "soc_estimator.h"

#define SOH_REST_CURRENT_MA               50
#define SOH_COMM_FAIL_THRESHOLD              3U

#define SOH_DELTA_GOOD_MV                 15U
#define SOH_DELTA_BAD_MV                  80U
#define SOH_WEAK_CELL_BAD_MV              60U

#define SOH_STATE_HEALTHY_MIN               90U
#define SOH_STATE_DEGRADED_MIN              75U
#define SOH_STATE_WARNING_MIN               60U

#define SOH_CAP_EMA_SHIFT                    2U  /* 容量 SOH 慢跟踪 */

#define SOH_CAP_FULL_CELL_MV              4150U
#define SOH_CAP_EMPTY_CELL_MV             3000U
#define SOH_CAP_TAPER_MA                   150
#define SOH_CAP_FLOW_MA                     50

#define SOH_CAP_LEARN_MIN_NUM                1U
#define SOH_CAP_LEARN_MIN_DEN                5U
#define SOH_CAP_LEARN_BLEND_NEW              3U
#define SOH_CAP_LEARN_BLEND_TOTAL           10U

static soh_status_t s_soh_status;
static uint32_t s_learned_capacity_mah;
static uint32_t s_cycle_count;
static bool s_capacity_valid;
static soh_capacity_learn_phase_t s_learn_phase;
static int64_t s_learn_accum_uah;
static int64_t s_learn_remainder;
static charge_state_t s_prev_charge_state;

static uint8_t soh_clamp_u8(int32_t value)
{
  if (value < 0)
  {
    return 0U;
  }
  if (value > 100)
  {
    return 100U;
  }
  return (uint8_t)value;
}

static uint8_t soh_lerp_penalty_u8(uint16_t value, uint16_t good_max, uint16_t bad_min,
                                   uint8_t good_score, uint8_t bad_score)
{
  if (value <= good_max)
  {
    return good_score;
  }
  if (value >= bad_min)
  {
    return bad_score;
  }

  return soh_clamp_u8((int32_t)good_score -
                      ((int32_t)(value - good_max) * (int32_t)(good_score - bad_score)) /
                      (int32_t)(bad_min - good_max));
}

static bool soh_inputs_resting(const bq76942_meas_t *meas)
{
  if (meas == NULL)
  {
    return false;
  }

  return (meas->current_ma <= SOH_REST_CURRENT_MA) &&
         (meas->current_ma >= -SOH_REST_CURRENT_MA);
}

static uint8_t Soh_EvalCapacitySoh(void)
{
  uint32_t design_mah = (uint32_t)BMS_NOMINAL_CAPACITY_MAH;

  if (!s_capacity_valid || (design_mah == 0U))
  {
    return 100U;
  }

  return soh_clamp_u8((int32_t)((s_learned_capacity_mah * 100U) / design_mah));
}

static uint8_t Soh_EvalConsistencySoh(uint16_t delta_mv, uint16_t weak_lag_mv, bool at_rest)
{
  uint8_t imbalance;
  uint8_t weak_cell;

  if (!at_rest)
  {
    return s_soh_status.consistency_soh != 0U ? s_soh_status.consistency_soh : 100U;
  }

  imbalance = soh_lerp_penalty_u8(delta_mv, SOH_DELTA_GOOD_MV, SOH_DELTA_BAD_MV, 100U, 40U);
  weak_cell = soh_lerp_penalty_u8(weak_lag_mv, 0U, SOH_WEAK_CELL_BAD_MV, 100U, 35U);

  return (imbalance < weak_cell) ? imbalance : weak_cell;
}

static uint8_t Soh_ApplyCapEma(uint8_t prev, uint8_t sample)
{
  if (prev == 0U)
  {
    return sample;
  }

  return (uint8_t)(((uint16_t)sample + ((uint16_t)prev * ((1U << SOH_CAP_EMA_SHIFT) - 1U))) >>
                   SOH_CAP_EMA_SHIFT);
}

static soh_state_t Soh_MapState(uint8_t soh_percent, bool valid)
{
  if (!valid)
  {
    return SOH_STATE_UNKNOWN;
  }
  if (soh_percent >= SOH_STATE_HEALTHY_MIN)
  {
    return SOH_STATE_HEALTHY;
  }
  if (soh_percent >= SOH_STATE_DEGRADED_MIN)
  {
    return SOH_STATE_DEGRADED;
  }
  if (soh_percent >= SOH_STATE_WARNING_MIN)
  {
    return SOH_STATE_WARNING;
  }
  return SOH_STATE_CRITICAL;
}

static uint16_t Soh_ComputeWeakCellLagMv(const bq76942_meas_t *meas)
{
  uint32_t sum = 0U;
  uint8_t i;

  for (i = 0U; i < BQ76942_CELL_COUNT; i++)
  {
    sum += meas->cell_mv[i];
  }

  return (uint16_t)(sum / BQ76942_CELL_COUNT - meas->vcell_min_mv);
}

static uint32_t soh_min_learn_mah(void)
{
  return (uint32_t)BMS_NOMINAL_CAPACITY_MAH * SOH_CAP_LEARN_MIN_NUM / SOH_CAP_LEARN_MIN_DEN;
}

static bool Soh_IsAtFull(const bq76942_meas_t *meas, charge_state_t charge_state)
{
  if (meas == NULL)
  {
    return false;
  }

  if (charge_state == CHARGE_STATE_COMPLETED)
  {
    return true;
  }

  return (meas->vcell_max_mv >= SOH_CAP_FULL_CELL_MV) &&
         (meas->current_ma < SOH_CAP_TAPER_MA);
}

static bool Soh_IsAtEmpty(const bq76942_meas_t *meas)
{
  if (meas == NULL)
  {
    return false;
  }

  return meas->vcell_min_mv <= SOH_CAP_EMPTY_CELL_MV;
}

static void Soh_LearnIntegrate(int16_t current_ma, uint32_t period_ms)
{
  int64_t numerator;

  if (period_ms == 0U)
  {
    return;
  }

  numerator = (int64_t)current_ma * (int64_t)period_ms + s_learn_remainder;
  s_learn_accum_uah += numerator / 3600LL;
  s_learn_remainder = numerator % 3600LL;
}

static void Soh_ApplyLearnedCapacityMah(uint32_t sample_mah)
{
  uint32_t design_mah = (uint32_t)BMS_NOMINAL_CAPACITY_MAH;
  uint32_t min_mah;
  uint32_t max_mah;
  uint32_t blended;

  if ((sample_mah == 0U) || (design_mah == 0U))
  {
    return;
  }

  min_mah = design_mah / 2U;
  max_mah = (design_mah * 11U) / 10U;
  if (sample_mah < min_mah)
  {
    sample_mah = min_mah;
  }
  if (sample_mah > max_mah)
  {
    sample_mah = max_mah;
  }

  if (!s_capacity_valid)
  {
    blended = sample_mah;
  }
  else
  {
    blended = ((sample_mah * SOH_CAP_LEARN_BLEND_NEW) +
               (s_learned_capacity_mah * (SOH_CAP_LEARN_BLEND_TOTAL - SOH_CAP_LEARN_BLEND_NEW))) /
              SOH_CAP_LEARN_BLEND_TOTAL;
  }

  s_learned_capacity_mah = blended;
  s_capacity_valid = true;
  s_soh_status.learned_capacity_mah = blended;
  s_soh_status.capacity_learned = true;
  s_cycle_count++;
  s_soh_status.cycle_count = s_cycle_count;
}

static void Soh_FinalizeCapacityLearn(void)
{
  uint32_t sample_mah = (uint32_t)(s_learn_accum_uah / 1000LL);

  if (sample_mah >= soh_min_learn_mah())
  {
    Soh_ApplyLearnedCapacityMah(sample_mah);
  }

  s_learn_phase = SOH_CAP_LEARN_IDLE;
  s_learn_accum_uah = 0LL;
  s_learn_remainder = 0LL;
}

static void Soh_CapacityLearnProcess(const soh_inputs_t *inputs, uint32_t period_ms)
{
  const bq76942_meas_t *meas = inputs->meas;
  bool at_full;
  bool at_empty;
  bool charging;
  bool discharging;

  if (meas == NULL)
  {
    return;
  }

  at_full = Soh_IsAtFull(meas, inputs->charge_state);
  at_empty = Soh_IsAtEmpty(meas);
  charging = (meas->current_ma > SOH_CAP_FLOW_MA) ||
             (inputs->charge_state == CHARGE_STATE_CHARGING);
  discharging = meas->current_ma < -SOH_CAP_FLOW_MA;

  switch (s_learn_phase)
  {
    case SOH_CAP_LEARN_IDLE:
      if (at_full)
      {
        s_learn_phase = SOH_CAP_LEARN_DISCHARGE;
        s_learn_accum_uah = 0LL;
        s_learn_remainder = 0LL;
      }
      else if (at_empty)
      {
        s_learn_phase = SOH_CAP_LEARN_CHARGE;
        s_learn_accum_uah = 0LL;
        s_learn_remainder = 0LL;
      }
      break;

    case SOH_CAP_LEARN_DISCHARGE:
      if (at_full && !discharging)
      {
        s_learn_accum_uah = 0LL;
        s_learn_remainder = 0LL;
        break;
      }

      if (discharging)
      {
        Soh_LearnIntegrate((int16_t)(-meas->current_ma), period_ms);
      }

      if (at_empty)
      {
        Soh_FinalizeCapacityLearn();
      }
      break;

    case SOH_CAP_LEARN_CHARGE:
      if (at_empty && !charging)
      {
        s_learn_accum_uah = 0LL;
        s_learn_remainder = 0LL;
        break;
      }

      if (charging)
      {
        Soh_LearnIntegrate(meas->current_ma, period_ms);
      }

      if (at_full ||
          ((s_prev_charge_state == CHARGE_STATE_CHARGING) &&
           (inputs->charge_state == CHARGE_STATE_COMPLETED)))
      {
        Soh_FinalizeCapacityLearn();
      }
      break;

    default:
      s_learn_phase = SOH_CAP_LEARN_IDLE;
      break;
  }

  s_soh_status.capacity_learn_phase = s_learn_phase;
  s_soh_status.capacity_learn_accum_mah = (uint32_t)(s_learn_accum_uah / 1000LL);
  s_prev_charge_state = inputs->charge_state;
}

static uint32_t Soh_CollectAlarms(const soh_inputs_t *inputs)
{
  uint32_t alarms = SOH_ALARM_NONE;

  if (inputs->comm_fail_count >= SOH_COMM_FAIL_THRESHOLD)
  {
    alarms |= SOH_ALARM_COMM;
  }

  if (inputs->bq_protect)
  {
    alarms |= SOH_ALARM_BQ_PROTECT;
  }

  if ((inputs->protect != NULL) &&
      (inputs->protect->state >= PWR_STATE_WARN))
  {
    alarms |= SOH_ALARM_THERMAL;
  }

  return alarms;
}

void Soh_Init(void)
{
  s_soh_status.state = SOH_STATE_UNKNOWN;
  s_soh_status.soh_percent = 0U;
  s_soh_status.valid = false;
  s_soh_status.capacity_soh = 100U;
  s_soh_status.consistency_soh = 100U;
  s_soh_status.delta_mv = 0U;
  s_soh_status.vmin_mv = 0U;
  s_soh_status.vmax_mv = 0U;
  s_soh_status.weak_cell_lag_mv = 0U;
  s_soh_status.chronic_factors = SOH_FACTOR_NONE;
  s_soh_status.alarm_flags = SOH_ALARM_NONE;
  s_soh_status.cycle_count = 0U;
  s_soh_status.learned_capacity_mah = (uint32_t)BMS_NOMINAL_CAPACITY_MAH;
  s_soh_status.capacity_learn_phase = SOH_CAP_LEARN_IDLE;
  s_soh_status.capacity_learn_accum_mah = 0U;
  s_soh_status.capacity_learned = false;

  s_learned_capacity_mah = (uint32_t)BMS_NOMINAL_CAPACITY_MAH;
  s_cycle_count = 0U;
  s_capacity_valid = false;
  s_learn_phase = SOH_CAP_LEARN_IDLE;
  s_learn_accum_uah = 0LL;
  s_learn_remainder = 0LL;
  s_prev_charge_state = CHARGE_STATE_IDLE;
}

void Soh_Process(const soh_inputs_t *inputs, uint32_t period_ms)
{
  uint32_t chronic = SOH_FACTOR_NONE;
  bool at_rest;

  if (inputs == NULL)
  {
    s_soh_status.valid = false;
    s_soh_status.state = SOH_STATE_UNKNOWN;
    return;
  }

  (void)inputs->temp;
  s_soh_status.alarm_flags = Soh_CollectAlarms(inputs);

  if ((inputs->meas == NULL) || (!inputs->meas->valid) ||
      ((s_soh_status.alarm_flags & SOH_ALARM_COMM) != 0U))
  {
    s_soh_status.valid = false;
    s_soh_status.state = SOH_STATE_UNKNOWN;
    return;
  }

  at_rest = soh_inputs_resting(inputs->meas);

  Soh_CapacityLearnProcess(inputs, period_ms);

  s_soh_status.vmin_mv = inputs->meas->vcell_min_mv;
  s_soh_status.vmax_mv = inputs->meas->vcell_max_mv;
  s_soh_status.delta_mv = (uint16_t)(inputs->meas->vcell_max_mv - inputs->meas->vcell_min_mv);
  s_soh_status.weak_cell_lag_mv = Soh_ComputeWeakCellLagMv(inputs->meas);
  s_soh_status.cycle_count = s_cycle_count;
  s_soh_status.learned_capacity_mah = s_learned_capacity_mah;

  s_soh_status.capacity_soh = Soh_EvalCapacitySoh();
  chronic |= SOH_FACTOR_CAPACITY;

  s_soh_status.consistency_soh =
      Soh_EvalConsistencySoh(s_soh_status.delta_mv, s_soh_status.weak_cell_lag_mv, at_rest);
  if (s_soh_status.delta_mv > SOH_DELTA_GOOD_MV)
  {
    chronic |= SOH_FACTOR_IMBALANCE;
  }
  if (s_soh_status.weak_cell_lag_mv > 0U)
  {
    chronic |= SOH_FACTOR_WEAK_CELL;
  }

  s_soh_status.chronic_factors = chronic;

  /* SOH = 容量保持率（慢 EMA）；一致性/保护/热告警不参与 */
  s_soh_status.soh_percent = Soh_ApplyCapEma(s_soh_status.soh_percent, s_soh_status.capacity_soh);
  s_soh_status.valid = true;
  s_soh_status.state = Soh_MapState(s_soh_status.soh_percent, true);
}

const soh_status_t *Soh_GetStatus(void)
{
  return &s_soh_status;
}

uint8_t Soh_GetPercent(void)
{
  return s_soh_status.soh_percent;
}

soh_state_t Soh_GetState(void)
{
  return s_soh_status.state;
}

bool Soh_IsValid(void)
{
  return s_soh_status.valid;
}
