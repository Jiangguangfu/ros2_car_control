/**
 * @file    charge_gate.c
 * @brief   充电安全仲裁：主因 + inhibit mask
 */
#include "charge_gate.h"

#include "app_freertos.h"
#include "bsp_power_rails.h"
#include "cell_voltage_protect.h"
#include "charge_manager.h"
#include "charge_path.h"
#include "lin_charger.h"

#define CHARGE_GATE_COMM_FAIL     3U
#define CHARGE_GATE_CELL_MIN_MV   2500U
#define CHARGE_GATE_CELL_SAFE_MV  4250U
#define CHARGE_GATE_FULL_EXIT_MV  4150U

static charge_reject_t charge_gate_from_fault(charge_fault_reason_t fault)
{
  switch (fault)
  {
    case CHARGE_FAULT_THERMAL:
      return CHARGE_REJECT_THERMAL;
    case CHARGE_FAULT_IMBALANCE:
      return CHARGE_REJECT_IMBALANCE;
    case CHARGE_FAULT_OVERVOLT:
      return CHARGE_REJECT_OVERVOLT;
    case CHARGE_FAULT_UNDERVOLT:
      return CHARGE_REJECT_UNDERVOLT;
    case CHARGE_FAULT_OVERCURRENT:
      return CHARGE_REJECT_OVERCURRENT;
    case CHARGE_FAULT_COMM:
      return CHARGE_REJECT_COMM;
    case CHARGE_FAULT_BQ_PROTECT:
      return CHARGE_REJECT_BQ_PROTECT;
    default:
      return CHARGE_REJECT_FAULT;
  }
}

static charge_reject_t charge_gate_pick(uint16_t mask, charge_fault_reason_t fault)
{
  if ((mask & CHG_INH_MEAS) != 0U)
  {
    return CHARGE_REJECT_MEAS;
  }
  if ((mask & CHG_INH_COMM) != 0U)
  {
    return CHARGE_REJECT_COMM;
  }
  if ((mask & CHG_INH_PROTECT) != 0U)
  {
    return CHARGE_REJECT_OVERCURRENT;
  }
  if ((mask & CHG_INH_THERMAL) != 0U)
  {
    const pwr_rails_status_t *pwr = BSP_PowerRails_GetStatus();

    if ((pwr != NULL) && (pwr->reason == PWR_REASON_COLD_CHARGE))
    {
      return CHARGE_REJECT_COLD;
    }
    return CHARGE_REJECT_THERMAL;
  }
  if ((mask & CHG_INH_VOLTAGE) != 0U)
  {
    if (CellVoltageProtect_IsCov())
    {
      return CHARGE_REJECT_OVERVOLT;
    }
    if (CellVoltageProtect_IsCuv())
    {
      return CHARGE_REJECT_UNDERVOLT;
    }
    return CHARGE_REJECT_OVERVOLT;
  }
  if ((mask & CHG_INH_IMBALANCE) != 0U)
  {
    return CHARGE_REJECT_IMBALANCE;
  }
  if ((mask & CHG_INH_LIN_COMM) != 0U)
  {
    return CHARGE_REJECT_LIN_COMM;
  }
  if ((mask & CHG_INH_FULL) != 0U)
  {
    return CHARGE_REJECT_FULL;
  }
  if ((mask & CHG_INH_FAULT) != 0U)
  {
    return charge_gate_from_fault(fault);
  }
  if ((mask & CHG_INH_LIN_SESSION) != 0U)
  {
    return CHARGE_REJECT_LIN_NOT_READY;
  }
  return CHARGE_REJECT_NONE;
}

void ChargeGate_Evaluate(bool require_charger, charge_gate_result_t *out)
{
  const bq76942_meas_t *meas;
  const pwr_rails_status_t *pwr;
  const charge_status_t *chg;
  const lin_charger_status_t *lin;
  uint16_t mask = 0U;
  charge_fault_reason_t fault = CHARGE_FAULT_NONE;

  if (out == NULL)
  {
    return;
  }

  out->code = CHARGE_REJECT_NONE;
  out->mask = 0U;

  meas = Bms_GetBqMeasurements();
  pwr = BSP_PowerRails_GetStatus();
  chg = ChargeManager_GetStatus();
  lin = LinCharger_GetStatus();
  mask = ChargePath_GetChargeInhibitMask();

  if (chg != NULL)
  {
    fault = chg->fault_reason;
  }

  if ((meas == NULL) || (!meas->valid))
  {
    mask = (uint16_t)(mask | CHG_INH_MEAS);
  }

  if (Bms_GetBqCommFailCount() >= CHARGE_GATE_COMM_FAIL)
  {
    mask = (uint16_t)(mask | CHG_INH_COMM);
  }

  if ((pwr != NULL) &&
      ((pwr->reason == PWR_REASON_SCD) || (pwr->reason == PWR_REASON_OCD) ||
       (pwr->reason == PWR_REASON_OCC) || (pwr->reason == PWR_REASON_SOFT_OCD)))
  {
    mask = (uint16_t)(mask | CHG_INH_PROTECT);
  }

  if ((pwr != NULL) &&
      ((pwr->state >= PWR_STATE_LIMIT) || (!pwr->sensor_ok)))
  {
    mask = (uint16_t)(mask | CHG_INH_THERMAL);
  }

  if ((meas != NULL) && meas->valid)
  {
    if (meas->vcell_min_mv < CHARGE_GATE_CELL_MIN_MV)
    {
      mask = (uint16_t)(mask | CHG_INH_VOLTAGE);
    }
    if (meas->vcell_max_mv >= CHARGE_GATE_CELL_SAFE_MV)
    {
      mask = (uint16_t)(mask | CHG_INH_VOLTAGE);
    }
  }

  if ((chg != NULL) && (chg->state == CHARGE_STATE_COMPLETED) &&
      (meas != NULL) && meas->valid &&
      (meas->vcell_max_mv >= CHARGE_GATE_FULL_EXIT_MV))
  {
    mask = (uint16_t)(mask | CHG_INH_FULL);
  }

  if ((chg != NULL) && (chg->state == CHARGE_STATE_FAULT))
  {
    mask = (uint16_t)(mask | CHG_INH_FAULT);
  }

  if (require_charger)
  {
    if ((lin == NULL) || (lin->session < LIN_SESSION_VI_OK) || lin->comm_lost)
    {
      mask = (uint16_t)(mask | CHG_INH_LIN_SESSION);
    }
  }

  if ((chg != NULL) && (chg->state == CHARGE_STATE_CHARGING) &&
      (!chg->charge_paused) && (!ChargePath_IsChargeInhibited()))
  {
    if (require_charger && ((mask & CHG_INH_LIN_SESSION) != 0U))
    {
      out->mask = (uint16_t)(mask & CHG_INH_LIN_SESSION);
      out->code = CHARGE_REJECT_LIN_NOT_READY;
      return;
    }
    out->code = CHARGE_REJECT_NONE;
    out->mask = 0U;
    return;
  }

  out->mask = mask;
  out->code = charge_gate_pick(mask, fault);
}
