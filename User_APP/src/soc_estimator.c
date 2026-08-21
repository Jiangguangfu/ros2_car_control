/**
 ******************************************************************************
 * @file    soc_estimator.c
 * @brief   库仑计 + 静置 OCV（充电只上修 / 放电只下修）+ 空满电稳定夹紧。
 ******************************************************************************
 */
#include "soc_estimator.h"

/* 6S NMC：单节电压 → SOC 线性近似（标定后可改） */
#define SOC_CELL_EMPTY_MV              3300U
#define SOC_CELL_FULL_MV               4200U
#define SOC_FULL_CLAMP_CELL_MV         4150U
#define SOC_FULL_TAPER_MA               150
#define SOC_REST_CURRENT_MA              50
#define SOC_REST_BLEND_DIV                5   /* 静置时向 OCV 收敛 1/5 步长 */
#define SOC_FULL_EMPTY_HOLD_MS         10000U /* 空/满电条件连续成立后再夹紧 */

/** 满充容量学习：充电电流 > 此值视为充电会话 */
#define SOC_LEARN_CHARGE_SESSION_MA         50
/** 满充容量学习：自低 SOC 起充才学习（%） */
#define SOC_LEARN_CHARGE_START_SOC_MAX      25U
/** 满充容量学习：单次充电增量超过标称此比例也可学习 */
#define SOC_LEARN_CHARGE_SESSION_MIN_PCT      50U

static soc_status_t s_soc_status;
static int64_t s_remaining_uah; /*剩余容量，单位：µAh*/
static int64_t s_coulomb_remainder;/*库仑计剩余容量，单位：µAh*/
static bool s_inited;
static int64_t s_learned_full_uah; /* 观测到的满充容量 µAh */
static bool s_charge_session_active;
static int64_t s_charge_session_start_uah;
static int64_t s_charge_session_added_uah;
static uint8_t s_charge_session_start_soc;
static int8_t s_last_flow;          /* +1 充电，-1 放电，0 未知 */
static uint32_t s_full_hold_ms;
static uint32_t s_empty_hold_ms;

static uint32_t soc_capacity_uah(void)
{
  return (uint32_t)BMS_NOMINAL_CAPACITY_MAH * 1000U;
}

static int32_t soc_clamp_i32(int32_t value, int32_t min_v, int32_t max_v)
{
  if (value < min_v)
  {
    return min_v;
  }
  if (value > max_v)
  {
    return max_v;
  }
  return value;
}

static int64_t soc_clamp_i64(int64_t value, int64_t min_v, int64_t max_v)
{
  if (value < min_v)
  {
    return min_v;
  }
  if (value > max_v)
  {
    return max_v;
  }
  return value;
}
/*电压转化为电量百分比*/
static uint8_t Soc_VoltagePercent(uint16_t vmin_mv)
{
  if (vmin_mv <= SOC_CELL_EMPTY_MV)
  {
    return 0U;
  }
  if (vmin_mv >= SOC_CELL_FULL_MV)
  {
    return 100U;
  }

  return (uint8_t)(((uint32_t)(vmin_mv - SOC_CELL_EMPTY_MV) * 100U) /
                   (SOC_CELL_FULL_MV - SOC_CELL_EMPTY_MV));
}

static void Soc_UpdateLearnedCap(void)
{
  const int64_t cap_uah = (int64_t)soc_capacity_uah();

  if (!s_soc_status.learned_cap_valid || s_learned_full_uah <= 0LL)
  {
    s_soc_status.learned_cap_pct = 100U;
    return;
  }

  s_soc_status.learned_cap_pct = (uint8_t)soc_clamp_i32(
      (int32_t)((s_learned_full_uah * 100LL) / cap_uah), 1, 100);
}

static void Soc_UpdateDerived(void)
{
  const int32_t cap_mah = (int32_t)BMS_NOMINAL_CAPACITY_MAH;

  s_soc_status.soc_nominal_mah = cap_mah;
  s_soc_status.soc_remaining_mah = (int32_t)(s_remaining_uah / 1000LL);
  s_soc_status.soc_percent = (uint8_t)soc_clamp_i32(
      (int32_t)((s_remaining_uah * 100LL) / (int64_t)soc_capacity_uah()),
      0, 100);
  Soc_UpdateLearnedCap();
}

static bool Soc_IsFullCharge(const bq76942_meas_t *meas)
{
  return (meas->vcell_max_mv >= SOC_FULL_CLAMP_CELL_MV) &&
         (meas->current_cc1_ma < SOC_FULL_TAPER_MA);
}

static void Soc_BeginChargeSession(int64_t remaining_before_delta)
{
  s_charge_session_active = true;
  s_charge_session_start_uah = remaining_before_delta;
  s_charge_session_added_uah = 0LL;
  s_charge_session_start_soc = s_soc_status.soc_percent;
}

static void Soc_TrackChargeSession(int64_t delta_uah, const bq76942_meas_t *meas)
{
  if (meas->current_cc1_ma > SOC_LEARN_CHARGE_SESSION_MA)
  {
    if (!s_charge_session_active)
    {
      Soc_BeginChargeSession(s_remaining_uah - delta_uah);
    }

    if (delta_uah > 0LL)
    {
      s_charge_session_added_uah += delta_uah;
    }
  }
  else
  {
    s_charge_session_active = false;
  }
}

static bool Soc_ChargeSessionCredibleForLearn(void)
{
  const int64_t cap_uah = (int64_t)soc_capacity_uah();
  const int64_t min_added_uah =
      (cap_uah * (int64_t)SOC_LEARN_CHARGE_SESSION_MIN_PCT) / 100LL;

  if (s_charge_session_start_soc <= SOC_LEARN_CHARGE_START_SOC_MAX)
  {
    return true;
  }

  return s_charge_session_added_uah >= min_added_uah;
}

static void Soc_LearnFullCapacity(int64_t observed_full_uah)
{
  const int64_t cap_uah = (int64_t)soc_capacity_uah();
  int64_t learned_uah = observed_full_uah;

  if (learned_uah <= 0LL)
  {
    return;
  }

  if (learned_uah > cap_uah)
  {
    learned_uah = cap_uah;
  }

  s_learned_full_uah = learned_uah;
  s_soc_status.learned_cap_valid = true;
}

static void Soc_TryLearnCapAtFull(const bq76942_meas_t *meas)
{
  int64_t observed_uah;

  if (!Soc_IsFullCharge(meas))
  {
    return;
  }

  if (!s_charge_session_active || !Soc_ChargeSessionCredibleForLearn())
  {
    return;
  }

  observed_uah = s_charge_session_start_uah + s_charge_session_added_uah;
  if (observed_uah > s_remaining_uah)
  {
    observed_uah = s_remaining_uah;
  }

  Soc_LearnFullCapacity(observed_uah);
}

static void Soc_SeedFromVoltage(uint16_t vmin_mv)
{
  const int64_t cap_uah = (int64_t)soc_capacity_uah();

  s_remaining_uah = ((int64_t)Soc_VoltagePercent(vmin_mv) * cap_uah) / 100LL;
  s_inited = true;
  s_soc_status.soc_valid = true;
  Soc_UpdateDerived();
}

static void Soc_UpdateFlow(const bq76942_meas_t *meas)
{
  if (meas->current_cc1_ma > SOC_REST_CURRENT_MA)
  {
    s_last_flow = 1;
  }
  else if (meas->current_cc1_ma < -SOC_REST_CURRENT_MA)
  {
    s_last_flow = -1;
  }
}

/* 空/满电条件连续成立一段时间后再夹紧，避免回弹跳变 */
static void Soc_ApplyFullEmptyClamp(const bq76942_meas_t *meas, uint32_t period_ms)
{
  const bool at_rest =
      (meas->current_cc1_ma <= SOC_REST_CURRENT_MA) &&
      (meas->current_cc1_ma >= -SOC_REST_CURRENT_MA);
  const bool full_cond =
      (meas->vcell_max_mv >= SOC_FULL_CLAMP_CELL_MV) &&
      (meas->current_cc1_ma < SOC_FULL_TAPER_MA);
  const bool empty_cond =
      at_rest && (meas->vcell_min_mv <= SOC_CELL_EMPTY_MV);

  if (full_cond)
  {
    if (s_full_hold_ms < SOC_FULL_EMPTY_HOLD_MS)
    {
      s_full_hold_ms += period_ms;
    }
  }
  else
  {
    s_full_hold_ms = 0U;
  }

  if (empty_cond)
  {
    if (s_empty_hold_ms < SOC_FULL_EMPTY_HOLD_MS)
    {
      s_empty_hold_ms += period_ms;
    }
  }
  else
  {
    s_empty_hold_ms = 0U;
  }

  if (s_full_hold_ms >= SOC_FULL_EMPTY_HOLD_MS)
  {
    s_remaining_uah = (int64_t)soc_capacity_uah();
  }

  if (s_empty_hold_ms >= SOC_FULL_EMPTY_HOLD_MS)
  {
    s_remaining_uah = 0LL;
  }
}

/* 静置 OCV：充电只向上修，放电只向下修 */
static void Soc_ApplyRestCorrection(const bq76942_meas_t *meas)
{
  int64_t target_uah;
  int64_t delta_uah;

  if ((meas->current_cc1_ma > SOC_REST_CURRENT_MA) ||
      (meas->current_cc1_ma < -SOC_REST_CURRENT_MA))
  {
    return;
  }

  if (s_last_flow == 0)
  {
    return;
  }

  target_uah = ((int64_t)Soc_VoltagePercent(meas->vcell_min_mv) *
                (int64_t)soc_capacity_uah()) / 100LL;
  delta_uah = (target_uah - s_remaining_uah) / SOC_REST_BLEND_DIV;

  if ((s_last_flow > 0) && (delta_uah < 0LL))
  {
    return;
  }

  if ((s_last_flow < 0) && (delta_uah > 0LL))
  {
    return;
  }

  s_remaining_uah += delta_uah;
}

void Soc_Init(void)
{
  s_soc_status.soc_percent = 0U;
  s_soc_status.soc_valid = false;
  s_soc_status.soc_remaining_mah = 0;
  s_soc_status.soc_nominal_mah = (int32_t)BMS_NOMINAL_CAPACITY_MAH;
  s_soc_status.learned_cap_pct = 100U;
  s_soc_status.learned_cap_valid = false;
  s_remaining_uah = 0LL;
  s_coulomb_remainder = 0LL;
  s_inited = false;
  s_learned_full_uah = 0LL;
  s_charge_session_active = false;
  s_charge_session_start_uah = 0LL;
  s_charge_session_added_uah = 0LL;
  s_charge_session_start_soc = 0U;
  s_last_flow = 0;
  s_full_hold_ms = 0U;
  s_empty_hold_ms = 0U;
}

void Soc_Process(const bq76942_meas_t *meas, uint32_t period_ms)
{
  int64_t delta_uah;/*本周期内电池剩余容量的变化量*/
  int64_t numerator;/*当前容量+上次剩余容量*/

  const int64_t cap_uah = (int64_t)soc_capacity_uah();

  if ((meas == NULL) || (!meas->valid) || (period_ms == 0U))
  {
    s_soc_status.soc_valid = false;
    return;
  }
  /*首次上电，根据静置电压初始化剩余容量*/
  if (!s_inited)
  {
    Soc_SeedFromVoltage(meas->vcell_min_mv);
    return;
  }

  /* 库仑计：BQ + 充电 / - 放电；delta_uah = mA * ms / 3600 */
  numerator = (int64_t)meas->current_cc1_ma * (int64_t)period_ms +
              s_coulomb_remainder;
  delta_uah = numerator / 3600LL;
  s_coulomb_remainder = numerator % 3600LL;
  s_remaining_uah += delta_uah;
  s_remaining_uah = soc_clamp_i64(s_remaining_uah, 0LL, cap_uah);
  Soc_UpdateFlow(meas);
  Soc_TrackChargeSession(delta_uah, meas);
  Soc_ApplyRestCorrection(meas);
  Soc_TryLearnCapAtFull(meas);
  Soc_ApplyFullEmptyClamp(meas, period_ms);
  s_remaining_uah = soc_clamp_i64(s_remaining_uah, 0LL, cap_uah);

  s_soc_status.soc_valid = true;
  Soc_UpdateDerived();
}

const soc_status_t *Soc_GetStatus(void)
{
  return &s_soc_status;
}

uint8_t Soc_GetPercent(void)
{
  return s_soc_status.soc_percent;
}

bool Soc_IsValid(void)
{
  return s_soc_status.soc_valid;
}

uint8_t Soc_GetLearnedCapPercent(void)
{
  return s_soc_status.learned_cap_pct;
}

bool Soc_IsLearnedCapValid(void)
{
  return s_soc_status.learned_cap_valid;
}
