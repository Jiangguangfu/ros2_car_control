/**
 ******************************************************************************
 * @file    low_power_consumption.c
 * @brief   BQ76942 RELAX/SLEEP 允许与退出控制。
 ******************************************************************************
 */
#include "low_power_consumption.h"
#include "bsp_ws2812.h"

#define LOW_POWER_LED_GREEN_LEVEL         64U

static low_power_state_t s_state;
static bool s_sleep_enabled;
static bool s_command_state_known;
static bool s_manual_disable;

static void LowPower_SetState(low_power_state_t state)
{
  if (state == s_state)
  {
    return;
  }

  s_state = state;
  if ((state == LOW_POWER_STATE_RELAX) ||
      (state == LOW_POWER_STATE_SLEEP))
  {
    BSP_WS2812_SetColor(0U, LOW_POWER_LED_GREEN_LEVEL, 0U);
  }
  else
  {
    BSP_WS2812_Off();
  }
}

static uint16_t LowPower_CurrentAbs(int16_t current_ma)
{
  int32_t current = current_ma;

  if (current < 0)
  {
    current = -current;
  }
  return (uint16_t)current;
}

void LowPower_Init(void)
{
  s_state = LOW_POWER_STATE_NORMAL;
  s_sleep_enabled = false;
  s_command_state_known = false;
  s_manual_disable = false;
  BSP_WS2812_Init();
  BSP_WS2812_Off();
}

bool LowPower_DisableSleep(I2C_HandleTypeDef *hi2c)
{
  bool ok;

  if (hi2c == NULL)
  {
    return false;
  }

  s_manual_disable = true;
  ok = BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_SLEEP_DISABLE);
  if (ok)
  {
    s_sleep_enabled = false;
    s_command_state_known = true;
    LowPower_SetState(LOW_POWER_STATE_NORMAL);
  }
  return ok;
}

void LowPower_EnableSleep(void)
{
  s_manual_disable = false;
  s_command_state_known = false;
}

void LowPower_Process(I2C_HandleTypeDef *hi2c,
                      const bq76942_meas_t *meas,
                      bool protection_valid,
                      bool protection_fault,
                      bool charger_connected)
{
  uint16_t battery_status;
  uint16_t current_abs_ma;
  bool block_sleep;
  low_power_state_t next_state;

  if (hi2c == NULL)
  {
    return;
  }

  current_abs_ma = ((meas != NULL) && meas->valid) ?
                       LowPower_CurrentAbs(meas->current_cc1_ma) :
                       UINT16_MAX;

  block_sleep = s_manual_disable ||
                (meas == NULL) || (!meas->valid) ||
                (!protection_valid) || protection_fault ||
                charger_connected ||
                (current_abs_ma >= BQ76942_SLEEP_CURRENT_MA);

  if (block_sleep)
  {
    if ((!s_command_state_known) || s_sleep_enabled)
    {
      if (BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_SLEEP_DISABLE))
      {
        s_sleep_enabled = false;
        s_command_state_known = true;
      }
    }
    LowPower_SetState(LOW_POWER_STATE_NORMAL);
    return;
  }

  /*
   * SLEEP_ENABLE 只允许睡眠，并不强制切换。CC1 低于 Sleep Current 时，
   * BQ 先进入 RELAX，再按芯片 Sleep Hysteresis Time 自动进入 SLEEP。
   */
  if ((!s_command_state_known) || (!s_sleep_enabled))
  {
    if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_SLEEP_ENABLE))
    {
      return;
    }
    s_sleep_enabled = true;
    s_command_state_known = true;
  }

  next_state = LOW_POWER_STATE_RELAX;
  if (BQ76942_ReadBatteryStatus(hi2c, &battery_status) &&
      ((battery_status & BQ76942_BATTERY_STATUS_SLEEP) != 0U))
  {
    next_state = LOW_POWER_STATE_SLEEP;
  }
  LowPower_SetState(next_state);
}

low_power_state_t LowPower_GetState(void)
{
  return s_state;
}
