/**
 ******************************************************************************
 * @file    bsp_power_rails.c
 * @brief   Multi-rail GPIO control + OC/SC protect policy.
 ******************************************************************************
 */
#include "bsp_power_rails.h"
#include "charge_path.h"
#include "app_freertos.h"
#include "main.h"

/* Software discharge OC (mA, negative current = discharge). Tune on hardware. */
#define PROTECT_SOFT_OCD_WARN_MA          12000  /* |Id| ≥ 12 A → drop 19V */
#define PROTECT_SOFT_OCD_FAULT_MA         20000  /* |Id| ≥ 20 A → all rails off */
#define PROTECT_SOFT_OCD_WARN_DEBOUNCE       3U  /* ~600 ms @ 200 ms */
#define PROTECT_SOFT_OCD_FAULT_DEBOUNCE      2U  /* ~400 ms */
#define PROTECT_SOFT_CLEAR_MA             2000   /* |Id| must fall below */

#define PROTECT_RAILS_WARN \
  ((uint8_t)(PWR_MASK_24V | PWR_MASK_12V | PWR_MASK_6V5 | PWR_MASK_5V))

static pwr_rails_status_t s_rails;
static protect_status_t s_protect;
static uint8_t s_soft_warn_count;
static uint8_t s_soft_fault_count;

/* -------------------------------------------------------------------------- */
/* GPIO / arbitration                                                         */
/* -------------------------------------------------------------------------- */

static void PowerRails_WriteGpio(pwr_rail_id_t rail, bool on)
{
  GPIO_PinState level = on ? GPIO_PIN_SET : GPIO_PIN_RESET;

  switch (rail)
  {
    case PWR_RAIL_24V:
      HAL_GPIO_WritePin(PWR_24V_BYPASS_EN_GPIO_Port, PWR_24V_BYPASS_EN_Pin, level);
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
      break;
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
      s_rails.rail_on[i] = on;
      continue;
    }

    on = ((enable_mask & (1u << i)) != 0U);
    s_rails.rail_on[i] = on;
    PowerRails_WriteGpio((pwr_rail_id_t)i, on);
  }

  s_rails.enabled_mask = 0U;
  for (i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    if (s_rails.rail_on[i])
    {
      s_rails.enabled_mask =
          (uint8_t)(s_rails.enabled_mask | (1u << i));
    }
  }
}

void BSP_PowerRails_Init(void)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    s_rails.rail_on[i] = false;
  }
  s_rails.enabled_mask = 0U;

  for (i = 0U; i < (uint8_t)PWR_REQ_COUNT; i++)
  {
    s_rails.request_mask[i] = PWR_MASK_ALL;
  }
}

void BSP_PowerRails_BootSequence(void)
{
  uint8_t i;

  PowerRails_DriveMask(0U);

  PowerRails_WriteGpio(PWR_RAIL_24V, true);
  s_rails.rail_on[PWR_RAIL_24V] = true;
  HAL_Delay(300);

  PowerRails_WriteGpio(PWR_RAIL_19V, true);
  s_rails.rail_on[PWR_RAIL_19V] = true;
  HAL_Delay(300);

  PowerRails_WriteGpio(PWR_RAIL_6V5, true);
  s_rails.rail_on[PWR_RAIL_6V5] = true;
  s_rails.rail_on[PWR_RAIL_5V] = true;
  HAL_Delay(200);

  PowerRails_WriteGpio(PWR_RAIL_12V, true);
  s_rails.rail_on[PWR_RAIL_12V] = true;

  s_rails.enabled_mask = PWR_MASK_ALL;
  for (i = 0U; i < (uint8_t)PWR_REQ_COUNT; i++)
  {
    s_rails.request_mask[i] = PWR_MASK_ALL;
  }
}

void BSP_PowerRails_SetRequest(pwr_req_source_t source, uint8_t enable_mask)
{
  if ((uint8_t)source >= (uint8_t)PWR_REQ_COUNT)
  {
    return;
  }

  s_rails.request_mask[source] = (uint8_t)(enable_mask & PWR_MASK_ALL);
}

void BSP_PowerRails_Apply(void)
{
  uint8_t mask = PWR_MASK_ALL;
  uint8_t i;

  for (i = 0U; i < (uint8_t)PWR_REQ_COUNT; i++)
  {
    mask = (uint8_t)(mask & s_rails.request_mask[i]);
  }

  PowerRails_DriveMask(mask);
}

void BSP_PowerRails_ApplyMask(uint8_t enable_mask)
{
  BSP_PowerRails_SetRequest(PWR_REQ_THERMAL, enable_mask);
  BSP_PowerRails_Apply();
}

const pwr_rails_status_t *BSP_PowerRails_GetStatus(void)
{
  return &s_rails;
}

/* -------------------------------------------------------------------------- */
/* Overcurrent / short-circuit protect                                        */
/* -------------------------------------------------------------------------- */

static void Protect_ApplyActuators(void)
{
  uint8_t pwr_mask = PWR_MASK_ALL;
  bool chg_off = false;
  bool dsg_off = false;

  switch (s_protect.state)
  {
    case PROTECT_STATE_WARN:
      pwr_mask = PROTECT_RAILS_WARN;
      break;

    case PROTECT_STATE_FAULT:
      switch (s_protect.reason)
      {
        case PROTECT_REASON_OCC:
          chg_off = true;
          pwr_mask = PWR_MASK_ALL;
          break;
        case PROTECT_REASON_SCD:
        case PROTECT_REASON_OCD:
        case PROTECT_REASON_SOFT_OCD:
        default:
          pwr_mask = 0U;
          chg_off = true;
          dsg_off = true;
          break;
      }
      break;

    case PROTECT_STATE_NORMAL:
    default:
      break;
  }

  s_protect.power_rails_mask = pwr_mask;
  s_protect.charge_inhibit = chg_off;
  s_protect.discharge_inhibit = dsg_off;

  BSP_PowerRails_SetRequest(PWR_REQ_PROTECT, pwr_mask);
  BSP_PowerRails_Apply();
  ChargePath_SetProtectInhibit(chg_off, dsg_off);
  ChargePath_Apply();
}

void Protect_Init(void)
{
  s_protect.state = PROTECT_STATE_NORMAL;
  s_protect.reason = PROTECT_REASON_NONE;
  s_protect.power_rails_mask = PWR_MASK_ALL;
  s_protect.charge_inhibit = false;
  s_protect.discharge_inhibit = false;
  s_protect.latched = false;
  s_protect.safety_ok = false;
  s_protect.status_a = 0U;
  s_protect.pack_current_ma = 0;
  s_soft_warn_count = 0U;
  s_soft_fault_count = 0U;

  BSP_PowerRails_SetRequest(PWR_REQ_PROTECT, PWR_MASK_ALL);
  ChargePath_SetProtectInhibit(false, false);
  Protect_ApplyActuators();
}

void Protect_Process(void)
{
  const bq76942_safety_t *safety = Bms_GetBqSafety();
  const bq76942_meas_t *meas = Bms_GetBqMeasurements();
  int16_t i_ma = 0;
  int16_t i_abs;
  protect_state_t next = PROTECT_STATE_NORMAL;
  protect_reason_t reason = PROTECT_REASON_NONE;

  if ((meas != NULL) && meas->valid)
  {
    i_ma = meas->current_ma;
  }
  s_protect.pack_current_ma = i_ma;
  i_abs = (i_ma < 0) ? (int16_t)(-i_ma) : i_ma;

  if ((safety != NULL) && safety->valid)
  {
    s_protect.safety_ok = true;
    s_protect.status_a = safety->status_a;

    if (safety->scd)
    {
      next = PROTECT_STATE_FAULT;
      reason = PROTECT_REASON_SCD;
      s_protect.latched = true;
    }
    else if (safety->ocd)
    {
      next = PROTECT_STATE_FAULT;
      reason = PROTECT_REASON_OCD;
      s_protect.latched = true;
    }
    else if (safety->occ)
    {
      next = PROTECT_STATE_FAULT;
      reason = PROTECT_REASON_OCC;
    }
  }
  else
  {
    s_protect.safety_ok = false;
  }

  if ((next != PROTECT_STATE_FAULT) || (reason == PROTECT_REASON_OCC))
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
        next = PROTECT_STATE_FAULT;
        reason = PROTECT_REASON_SOFT_OCD;
        s_protect.latched = true;
      }
      else if ((s_soft_warn_count >= PROTECT_SOFT_OCD_WARN_DEBOUNCE) &&
               (next < PROTECT_STATE_WARN))
      {
        next = PROTECT_STATE_WARN;
        reason = PROTECT_REASON_SOFT_OCD;
      }
    }
    else
    {
      s_soft_warn_count = 0U;
      s_soft_fault_count = 0U;
    }
  }

  if (s_protect.latched &&
      ((s_protect.reason == PROTECT_REASON_SCD) ||
       (s_protect.reason == PROTECT_REASON_OCD) ||
       (s_protect.reason == PROTECT_REASON_SOFT_OCD)))
  {
    next = PROTECT_STATE_FAULT;
    reason = s_protect.reason;
  }

  if ((next == PROTECT_STATE_NORMAL) &&
      (s_protect.reason == PROTECT_REASON_OCC) &&
      (!s_protect.latched))
  {
    reason = PROTECT_REASON_NONE;
  }

  s_protect.state = next;
  s_protect.reason = reason;
  Protect_ApplyActuators();
}

const protect_status_t *Protect_GetStatus(void)
{
  return &s_protect;
}

protect_state_t Protect_GetState(void)
{
  return s_protect.state;
}

bool Protect_ClearFault(void)
{
  const bq76942_safety_t *safety = Bms_GetBqSafety();
  const bq76942_meas_t *meas = Bms_GetBqMeasurements();
  int16_t i_abs;

  if (s_protect.state != PROTECT_STATE_FAULT)
  {
    return false;
  }

  if ((safety == NULL) || (!safety->valid))
  {
    return false;
  }

  if (safety->scd || safety->ocd || safety->occ)
  {
    return false;
  }

  if ((meas == NULL) || (!meas->valid))
  {
    return false;
  }

  i_abs = (meas->current_ma < 0) ?
          (int16_t)(-meas->current_ma) : meas->current_ma;
  if (i_abs > PROTECT_SOFT_CLEAR_MA)
  {
    return false;
  }

  s_protect.latched = false;
  s_protect.state = PROTECT_STATE_NORMAL;
  s_protect.reason = PROTECT_REASON_NONE;
  s_soft_warn_count = 0U;
  s_soft_fault_count = 0U;
  Protect_ApplyActuators();
  return true;
}
