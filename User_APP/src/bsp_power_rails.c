/**
 ******************************************************************************
 * @file    bsp_power_rails.c
 * @brief   Multi-rail GPIO enable control for BMS board outputs.
 ******************************************************************************
 */
#include "bsp_power_rails.h"
#include "main.h"

static pwr_rails_status_t s_status;

static void BSP_PowerRails_WriteGpio(pwr_rail_id_t rail, bool on)
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
      /* No GPIO — state tracked only; 5V LDO fed from 6.5V rail. */
      break;
    default:
      break;
  }
}

static void BSP_PowerRails_UpdateMask(void)
{
  uint8_t mask = 0U;
  uint8_t i;

  for (i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    if (s_status.rail_on[i])
    {
      mask = (uint8_t)(mask | (1u << i));
    }
  }
  s_status.enabled_mask = mask;
}

void BSP_PowerRails_Init(void)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    s_status.rail_on[i] = false;
  }
  s_status.enabled_mask = 0U;
}

void BSP_PowerRails_BootSequence(void)
{
  /* Match legacy main.c order: 24V → 19V → 6.5V → 12V. */
  BSP_PowerRails_ApplyMask(0U);

  BSP_PowerRails_WriteGpio(PWR_RAIL_24V, true);
  s_status.rail_on[PWR_RAIL_24V] = true;
  HAL_Delay(300);

  BSP_PowerRails_WriteGpio(PWR_RAIL_19V, true);
  s_status.rail_on[PWR_RAIL_19V] = true;
  HAL_Delay(300);

  BSP_PowerRails_WriteGpio(PWR_RAIL_6V5, true);
  s_status.rail_on[PWR_RAIL_6V5] = true;
  s_status.rail_on[PWR_RAIL_5V] = true;
  HAL_Delay(200);

  BSP_PowerRails_WriteGpio(PWR_RAIL_12V, true);
  s_status.rail_on[PWR_RAIL_12V] = true;

  BSP_PowerRails_UpdateMask();
}

void BSP_PowerRails_ApplyMask(uint8_t enable_mask)
{
  uint8_t i;
  bool on_6v5;

  on_6v5 = ((enable_mask & PWR_MASK_6V5) != 0U);

  for (i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    bool on;

    if (i == (uint8_t)PWR_RAIL_5V)
    {
      on = on_6v5 && ((enable_mask & PWR_MASK_5V) != 0U);
      s_status.rail_on[i] = on;
      continue;
    }

    on = ((enable_mask & (1u << i)) != 0U);
    s_status.rail_on[i] = on;
    BSP_PowerRails_WriteGpio((pwr_rail_id_t)i, on);
  }

  BSP_PowerRails_UpdateMask();
}

const pwr_rails_status_t *BSP_PowerRails_GetStatus(void)
{
  return &s_status;
}
