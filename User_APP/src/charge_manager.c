/**
 ******************************************************************************
 * @file    charge_manager.c
 * @brief   充电状态机：CC/CV 管理、启动/停止、满电退出。
 ******************************************************************************
 */
#include "charge_manager.h"
#include "charge_path.h"
#include "bsp_power_rails.h"
#include "cell_balance_manager.h"
#include "bq76942.h"
#include "app_freertos.h"
#include "cmsis_os2.h"

/* 6S NMC 阈值，可按电芯规格标定 */
#define CHARGE_CELL_CV_ENTER_MV           4150U  /* CC → CV */
#define CHARGE_CELL_FULL_MV               4200U  /* 满电目标 */
#define CHARGE_CELL_FULL_EXIT_MV          4150U  /* 已完成后再充电压门限 */
#define CHARGE_CELL_MIN_MV                2500U
#define CHARGE_CELL_MAX_SAFE_MV           4250U

#define CHARGE_CC_DETECT_CURRENT_MA         50   /* 判定有充电电流 */
#define CHARGE_CV_TAPER_CURRENT_MA         150   /* CV 终止电流 */
#define CHARGE_CV_TAPER_DEBOUNCE             6U  /* ~3 s @ 500 ms */
#define CHARGE_CC_TIMEOUT_MS          (7200000U) /* CC 阶段最长 2 h */
#define CHARGE_CV_TIMEOUT_MS          (3600000U) /* CV 阶段最长 1 h */
#define CHARGE_MAX_CURRENT_MA             3000   /* 过流保护 */
#define CHARGE_COMM_FAIL_THRESHOLD           3U
#define CHARGE_START_DEBOUNCE               20U  /* ~10 s @ 500 ms；给 LIN 桩出流时间 */

#define CHARGE_TICK_MS                     500U

static charge_status_t s_status;
static uint32_t s_phase_enter_ms;
static uint8_t s_start_debounce;
static bool s_inited;
static bool s_lin_charge_expect;

static void ChargeManager_SetFault(charge_fault_reason_t reason)
{
  s_status.state = CHARGE_STATE_FAULT;
  s_status.phase = CHARGE_PHASE_NONE;
  s_status.fault_reason = reason;
  s_status.user_start_request = false;
  s_status.charge_paused = false;
  s_status.cv_taper_count = 0U;
  ChargePath_SetChargeManagerInhibit(true);
  ChargePath_Apply();
  Balance_SetChargerPresent(false);
}

static void ChargeManager_EnterIdle(void)
{
  s_status.state = CHARGE_STATE_IDLE;
  s_status.phase = CHARGE_PHASE_NONE;
  s_status.fault_reason = CHARGE_FAULT_NONE;
  s_status.user_start_request = false;
  s_status.charge_paused = false;
  s_status.cv_taper_count = 0U;
  s_status.charge_elapsed_ms = 0U;
  s_phase_enter_ms = 0U;
  s_start_debounce = 0U;
  ChargePath_SetChargeManagerInhibit(true);
  ChargePath_Apply();
  Balance_SetChargerPresent(false);
}

static void ChargeManager_EnterCompleted(void)
{
  s_status.state = CHARGE_STATE_COMPLETED;
  s_status.phase = CHARGE_PHASE_NONE;
  s_status.fault_reason = CHARGE_FAULT_NONE;
  s_status.user_start_request = false;
  s_status.charge_paused = false;
  s_status.cv_taper_count = 0U;
  ChargePath_SetChargeManagerInhibit(true);
  ChargePath_Apply();
  Balance_SetChargerPresent(true);
}

static bool ChargeManager_PreStartChecks(void)
{
  const bq76942_meas_t *meas = Bms_GetBqMeasurements();
  const pwr_rails_status_t *thermal = BSP_PowerRails_GetStatus();
  const uint32_t comm_fail = Bms_GetBqCommFailCount();

  if ((meas == NULL) || (!meas->valid))
  {
    return false;
  }

  if (comm_fail >= CHARGE_COMM_FAIL_THRESHOLD)
  {
    return false;
  }

  if ((thermal != NULL) &&
      ((thermal->state >= PWR_STATE_LIMIT) || (!thermal->sensor_ok)))
  {
    return false;
  }

  if (meas->vcell_min_mv < CHARGE_CELL_MIN_MV)
  {
    return false;
  }

  if (meas->vcell_max_mv >= CHARGE_CELL_MAX_SAFE_MV)
  {
    return false;
  }

  return true;
}

static bool ChargeManager_EnsureChargeFet(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return false;
  }

  if (BQ76942_EnableChargePath(hi2c))
  {
    s_status.chg_fet_on = true;
    return true;
  }

  s_status.chg_fet_on = false;
  return false;
}

static void ChargeManager_ApplyChargePath(bool allow)
{
  ChargePath_SetChargeManagerInhibit(!allow);
  ChargePath_Apply();
  s_status.charge_allowed = allow && (!ChargePath_IsChargeInhibited());
  s_status.charge_paused = (s_status.state == CHARGE_STATE_CHARGING) &&
                           (!s_status.charge_allowed);
}

static void ChargeManager_UpdateSamples(void)
{
  const bq76942_meas_t *meas = Bms_GetBqMeasurements();

  if ((meas == NULL) || (!meas->valid))
  {
    return;
  }

  s_status.vcell_min_mv = meas->vcell_min_mv;
  s_status.vcell_max_mv = meas->vcell_max_mv;
  s_status.pack_mv = meas->pack_mv;
  s_status.pack_current_ma = meas->current_ma;
}

static bool ChargeManager_CheckProtectFault(I2C_HandleTypeDef *hi2c)
{
  bool protect = false;

  if ((hi2c != NULL) && BQ76942_ReadSafetyStatus(hi2c, &protect) && protect)
  {
    ChargeManager_SetFault(CHARGE_FAULT_BQ_PROTECT);
    return true;
  }

  return false;
}

static void ChargeManager_CheckFaults(I2C_HandleTypeDef *hi2c)
{
  const pwr_rails_status_t *thermal = BSP_PowerRails_GetStatus();
  const uint32_t comm_fail = Bms_GetBqCommFailCount();
  const bq76942_meas_t *meas = Bms_GetBqMeasurements();

  if (ChargeManager_CheckProtectFault(hi2c))
  {
    return;
  }

  if (comm_fail >= CHARGE_COMM_FAIL_THRESHOLD)
  {
    ChargeManager_SetFault(CHARGE_FAULT_COMM);
    return;
  }

  if ((meas == NULL) || (!meas->valid))
  {
    return;
  }

  if (meas->vcell_max_mv >= CHARGE_CELL_MAX_SAFE_MV)
  {
    ChargeManager_SetFault(CHARGE_FAULT_OVERVOLT);
    return;
  }

  if (meas->vcell_min_mv < CHARGE_CELL_MIN_MV)
  {
    ChargeManager_SetFault(CHARGE_FAULT_UNDERVOLT);
    return;
  }

  if (meas->current_ma > CHARGE_MAX_CURRENT_MA)
  {
    ChargeManager_SetFault(CHARGE_FAULT_OVERCURRENT);
    return;
  }

  if ((thermal != NULL) && (thermal->state == PWR_STATE_FAULT))
  {
    ChargeManager_SetFault(CHARGE_FAULT_THERMAL);
    return;
  }

  /* 压差 / LIN 超时：charge_path 已关 CFET，保持 CHARGING 态等待恢复 */
  s_status.charge_paused =
      ChargePath_IsImbalanceChargeInhibit() || ChargePath_IsLinCommInhibit();
}

static void ChargeManager_ProcessCharging(I2C_HandleTypeDef *hi2c)
{
  const uint32_t now_ms = osKernelGetTickCount();
  const uint32_t phase_ms = now_ms - s_phase_enter_ms;

  s_status.charge_elapsed_ms += CHARGE_TICK_MS;

  ChargeManager_ApplyChargePath(true);
  (void)ChargeManager_EnsureChargeFet(hi2c);
  Balance_SetChargerPresent(true);

  if (s_status.charge_paused || ChargePath_IsChargeInhibited())
  {
    /* thermal / 压差 / LIN 超时暂停，不改变 CC/CV 阶段 */
    if ((BSP_PowerRails_GetState() == PWR_STATE_FAULT) ||
        ChargePath_IsImbalanceChargeInhibit())
    {
      if (BSP_PowerRails_GetState() == PWR_STATE_FAULT)
      {
        ChargeManager_SetFault(CHARGE_FAULT_THERMAL);
      }
    }
    return;
  }

  if (s_status.phase == CHARGE_PHASE_CC)
  {
    if (s_status.vcell_max_mv >= CHARGE_CELL_CV_ENTER_MV)
    {
      s_status.phase = CHARGE_PHASE_CV;
      s_status.cv_taper_count = 0U;
      s_phase_enter_ms = now_ms;
    }
    else if (phase_ms >= CHARGE_CC_TIMEOUT_MS)
    {
      ChargeManager_SetFault(CHARGE_FAULT_TIMEOUT);
      return;
    }
    else if (s_start_debounce >= CHARGE_START_DEBOUNCE)
    {
      /* LIN 桩在 BMS 报 CHARGING 之后才启动 BQ25756，不能按实验室电源立刻判无流。 */
      if ((!s_lin_charge_expect) &&
          (s_status.pack_current_ma < CHARGE_CC_DETECT_CURRENT_MA))
      {
        ChargeManager_SetFault(CHARGE_FAULT_NO_CURRENT);
        return;
      }
    }
    else
    {
      s_start_debounce++;
    }
  }

  if (s_status.phase == CHARGE_PHASE_CV)
  {
    if (phase_ms >= CHARGE_CV_TIMEOUT_MS)
    {
      ChargeManager_SetFault(CHARGE_FAULT_TIMEOUT);
      return;
    }

    if ((s_status.vcell_max_mv >= (CHARGE_CELL_FULL_MV - 20U)) &&
        (s_status.pack_current_ma <= CHARGE_CV_TAPER_CURRENT_MA))
    {
      if (s_status.cv_taper_count < 255U)
      {
        s_status.cv_taper_count++;
      }
    }
    else
    {
      s_status.cv_taper_count = 0U;
    }

    if (s_status.cv_taper_count >= CHARGE_CV_TAPER_DEBOUNCE)
    {
      ChargeManager_EnterCompleted();
    }
  }
}

void ChargeManager_Init(void)
{
  if (s_inited)
  {
    return;
  }

  ChargeManager_EnterIdle();
  s_inited = true;
}

void ChargeManager_SetLinChargeExpect(bool expect)
{
  s_lin_charge_expect = expect;
}

bool ChargeManager_Start(void)
{
  if (s_status.state == CHARGE_STATE_CHARGING)
  {
    return true;
  }

  if (s_status.state == CHARGE_STATE_FAULT)
  {
    if (!ChargeManager_ClearFault())
    {
      return false;
    }
  }

  if (s_status.state == CHARGE_STATE_COMPLETED)
  {
    const bq76942_meas_t *meas = Bms_GetBqMeasurements();

    if ((meas == NULL) || (!meas->valid) ||
        (meas->vcell_max_mv >= CHARGE_CELL_FULL_EXIT_MV))
    {
      return false;
    }
  }

  if (!ChargeManager_PreStartChecks())
  {
    return false;
  }

  s_status.state = CHARGE_STATE_CHARGING;
  s_status.phase = CHARGE_PHASE_CC;
  s_status.fault_reason = CHARGE_FAULT_NONE;
  s_status.user_start_request = true;
  s_status.charge_paused = false;
  s_status.cv_taper_count = 0U;
  s_status.charge_elapsed_ms = 0U;
  s_phase_enter_ms = osKernelGetTickCount();
  s_start_debounce = 0U;

  ChargeManager_ApplyChargePath(true);
  Balance_SetChargerPresent(true);
  return true;
}

void ChargeManager_Stop(void)
{
  if (s_status.state == CHARGE_STATE_IDLE)
  {
    return;
  }

  if (s_status.state == CHARGE_STATE_CHARGING)
  {
    s_status.fault_reason = CHARGE_FAULT_USER_STOP;
  }

  ChargeManager_EnterIdle();
}

bool ChargeManager_ClearFault(void)
{
  const pwr_rails_status_t *thermal = BSP_PowerRails_GetStatus();
  const uint32_t comm_fail = Bms_GetBqCommFailCount();

  if (s_status.state != CHARGE_STATE_FAULT)
  {
    return false;
  }

  if (comm_fail >= CHARGE_COMM_FAIL_THRESHOLD)
  {
    return false;
  }

  if ((thermal != NULL) &&
      ((thermal->state == PWR_STATE_FAULT) || (!thermal->sensor_ok)))
  {
    return false;
  }

  if (Bms_GetBqMeasurements() == NULL)
  {
    return false;
  }

  /* 不可恢复类故障需外部干预后再清 */
  if ((s_status.fault_reason == CHARGE_FAULT_OVERVOLT) ||
      (s_status.fault_reason == CHARGE_FAULT_BQ_PROTECT))
  {
    return false;
  }

  ChargeManager_EnterIdle();
  return true;
}

void ChargeManager_Process(I2C_HandleTypeDef *hi2c)
{
  if (!s_inited)
  {
    ChargeManager_Init();
  }

  ChargeManager_UpdateSamples();

  switch (s_status.state)
  {
    case CHARGE_STATE_IDLE:
      ChargeManager_ApplyChargePath(false);
      break;

    case CHARGE_STATE_COMPLETED:
      ChargeManager_ApplyChargePath(false);
      break;

    case CHARGE_STATE_CHARGING:
      ChargeManager_CheckFaults(hi2c);
      if (s_status.state == CHARGE_STATE_FAULT)
      {
        break;
      }
      ChargeManager_ProcessCharging(hi2c);
      break;

    case CHARGE_STATE_FAULT:
      ChargeManager_ApplyChargePath(false);
      break;

    default:
      ChargeManager_EnterIdle();
      break;
  }

  if ((hi2c != NULL) && (s_status.state != CHARGE_STATE_IDLE))
  {
    uint8_t fet = 0U;

    if (BQ76942_ReadFetStatus(hi2c, &fet))
    {
      s_status.chg_fet_on = ((fet & BQ76942_FETSTAT_CHG_FET) != 0U);
    }
  }
}

const charge_status_t *ChargeManager_GetStatus(void)
{
  return &s_status;
}

charge_state_t ChargeManager_GetState(void)
{
  return s_status.state;
}

charge_phase_t ChargeManager_GetPhase(void)
{
  return s_status.phase;
}
