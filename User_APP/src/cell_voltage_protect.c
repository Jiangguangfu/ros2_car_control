/**
 ******************************************************************************
 * @file    cell_voltage_protect.c
 * @brief   BQ76942 COV/CUV inhibit + vmin low-battery warning.
 ******************************************************************************
 */
#include "cell_voltage_protect.h"
#include "charge_path.h"

static cell_voltage_protect_state_t s_state = VOLTPROT_STATE_NORMAL;
static uint8_t s_status_a;
static bool s_cov;
static bool s_cuv;
static bool s_low_voltage_warn;
static bool s_charge_inhibit;
static bool s_discharge_inhibit;
static bool s_valid;

void CellVoltageProtect_Init(void)
{
  s_state = VOLTPROT_STATE_NORMAL;
  s_status_a = 0U;
  s_cov = false;
  s_cuv = false;
  s_low_voltage_warn = false;
  s_charge_inhibit = false;
  s_discharge_inhibit = false;
  s_valid = false;
  ChargePath_SetVoltageInhibit(false, false);
}

void CellVoltageProtect_Process(uint8_t status_a, bool bq_valid,
                                const bq76942_meas_t *meas)
{
  bool cov = false;
  bool cuv = false;
  bool charge_off = false;
  bool discharge_off = false;
  bool meas_valid = ((meas != NULL) && meas->valid);
  cell_voltage_protect_state_t state = VOLTPROT_STATE_NORMAL;

  s_status_a = status_a;

  if (bq_valid)
  {
    cov = ((status_a & BQ76942_SA_COV) != 0U);
    cuv = ((status_a & BQ76942_SA_CUV) != 0U);
  }

  charge_off = cov;
  discharge_off = cuv;

  if (cov)
  {
    state = VOLTPROT_STATE_COV;
  }
  else if (cuv)
  {
    state = VOLTPROT_STATE_CUV;
  }

  if (meas_valid)
  {
    if (meas->vcell_min_mv <= VOLTPROT_LOW_WARN_MV)
    {
      s_low_voltage_warn = true;
    }
    else if (meas->vcell_min_mv >= VOLTPROT_LOW_WARN_CLEAR_MV)
    {
      s_low_voltage_warn = false;
    }

    if ((state == VOLTPROT_STATE_NORMAL) && s_low_voltage_warn)
    {
      state = VOLTPROT_STATE_LOW_WARN;
    }
  }

  s_state = state;
  s_cov = cov;
  s_cuv = cuv;
  s_charge_inhibit = charge_off;
  s_discharge_inhibit = discharge_off;
  s_valid = bq_valid || meas_valid;

  ChargePath_SetVoltageInhibit(charge_off, discharge_off);
}

cell_voltage_protect_state_t CellVoltageProtect_GetState(void)
{
  return s_state;
}

uint8_t CellVoltageProtect_GetStatusA(void)
{
  return s_status_a;
}

bool CellVoltageProtect_IsCov(void)
{
  return s_cov;
}

bool CellVoltageProtect_IsCuv(void)
{
  return s_cuv;
}

bool CellVoltageProtect_IsLowVoltageWarn(void)
{
  return s_low_voltage_warn;
}

bool CellVoltageProtect_IsChargeInhibited(void)
{
  return s_charge_inhibit;
}

bool CellVoltageProtect_IsDischargeInhibited(void)
{
  return s_discharge_inhibit;
}

bool CellVoltageProtect_IsValid(void)
{
  return s_valid;
}
