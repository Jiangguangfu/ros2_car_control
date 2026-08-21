/**
 ******************************************************************************
 * @file    charge_path.c
 * @brief   CFETOFF/DFETOFF arbitration for thermal + imbalance sources.
 ******************************************************************************
 */
#include "charge_path.h"
#include "main.h"

static bool s_thermal_charge_off;
static bool s_thermal_discharge_off;
static bool s_imbalance_charge_off;
static bool s_charge_manager_off;
static bool s_protect_charge_off;
static bool s_protect_discharge_off;
static bool s_voltage_charge_off;
static bool s_voltage_discharge_off;
static bool s_lin_comm_charge_off;
static bool s_boot_discharge_off;
static bool s_charge_inhibited;
static bool s_discharge_inhibited;

void ChargePath_Init(void)
{
  static bool s_inited;

  if (!s_inited)
  {
    s_thermal_charge_off = false;
    s_thermal_discharge_off = false;
    s_imbalance_charge_off = false;
    s_charge_manager_off = true;
    s_protect_charge_off = false;
    s_protect_discharge_off = false;
    s_voltage_charge_off = false;
    s_voltage_discharge_off = false;
    s_lin_comm_charge_off = false;
    s_boot_discharge_off = true;
    s_charge_inhibited = false;
    s_discharge_inhibited = false;
    s_inited = true;
  }

  ChargePath_Apply();
}

void ChargePath_SetThermalInhibit(bool charge_off, bool discharge_off)
{
  s_thermal_charge_off = charge_off;
  s_thermal_discharge_off = discharge_off;
}

void ChargePath_SetImbalanceChargeInhibit(bool charge_off)
{
  s_imbalance_charge_off = charge_off;
}

void ChargePath_SetChargeManagerInhibit(bool charge_off)
{
  s_charge_manager_off = charge_off;
}

void ChargePath_SetProtectInhibit(bool charge_off, bool discharge_off)
{
  s_protect_charge_off = charge_off;
  s_protect_discharge_off = discharge_off;
}

void ChargePath_SetVoltageInhibit(bool charge_off, bool discharge_off)
{
  s_voltage_charge_off = charge_off;
  s_voltage_discharge_off = discharge_off;
}

void ChargePath_SetLinCommInhibit(bool charge_off)
{
  s_lin_comm_charge_off = charge_off;
}

void ChargePath_SetBootDischargeInhibit(bool discharge_off)
{
  s_boot_discharge_off = discharge_off;
}

void ChargePath_Apply(void)
{
  s_charge_inhibited =
      s_thermal_charge_off || s_imbalance_charge_off || s_charge_manager_off ||
      s_protect_charge_off || s_voltage_charge_off || s_lin_comm_charge_off;
  s_discharge_inhibited =
      s_thermal_discharge_off || s_protect_discharge_off ||
      s_voltage_discharge_off || s_boot_discharge_off;

  /* Active-high: SET forces corresponding FET path off. */
  HAL_GPIO_WritePin(BQ_CFETOFF_GPIO_Port, BQ_CFETOFF_Pin,
                    s_charge_inhibited ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BQ_DFETOFF_GPIO_Port, BQ_DFETOFF_Pin,
                    s_discharge_inhibited ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool ChargePath_IsImbalanceChargeInhibit(void)
{
  return s_imbalance_charge_off;
}

bool ChargePath_IsLinCommInhibit(void)
{
  return s_lin_comm_charge_off;
}

bool ChargePath_IsChargeInhibited(void)
{
  return s_charge_inhibited;
}

bool ChargePath_IsDischargeInhibited(void)
{
  return s_discharge_inhibited;
}

uint16_t ChargePath_GetChargeInhibitMask(void)
{
  uint16_t mask = 0U;

  if (s_thermal_charge_off)
  {
    mask = (uint16_t)(mask | CHG_INH_THERMAL);
  }
  if (s_protect_charge_off)
  {
    mask = (uint16_t)(mask | CHG_INH_PROTECT);
  }
  if (s_voltage_charge_off)
  {
    mask = (uint16_t)(mask | CHG_INH_VOLTAGE);
  }
  if (s_imbalance_charge_off)
  {
    mask = (uint16_t)(mask | CHG_INH_IMBALANCE);
  }
  if (s_lin_comm_charge_off)
  {
    mask = (uint16_t)(mask | CHG_INH_LIN_COMM);
  }

  return mask;
}
