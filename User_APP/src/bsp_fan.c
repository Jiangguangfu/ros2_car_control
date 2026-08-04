/**
 ******************************************************************************
 * @file    bsp_fan.c
 * @brief   System fan on TIM4_CH3 (SYS_FAN_PWM, PB8). ARR=3839.
 ******************************************************************************
 */
#include "bsp_fan.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

#define FAN_TIM_CHANNEL   TIM_CHANNEL_3
#define FAN_PWM_PERIOD    3839U

static uint8_t s_duty_percent;

void BSP_Fan_Init(void)
{
  s_duty_percent = 0U;
  if (htim4.State == HAL_TIM_STATE_RESET) {
    return;
  }
  __HAL_TIM_SET_COMPARE(&htim4, FAN_TIM_CHANNEL, 0U);
  (void)HAL_TIM_PWM_Start(&htim4, FAN_TIM_CHANNEL);
}

void BSP_Fan_SetDutyPercent(uint8_t duty_percent)
{
  uint32_t pulse;

  if (htim4.State == HAL_TIM_STATE_RESET) {
    return;
  }

  if (duty_percent > 100U)
  {
    duty_percent = 100U;
  }

  s_duty_percent = duty_percent;
  pulse = ((uint32_t)FAN_PWM_PERIOD * (uint32_t)duty_percent) / 100U;
  if ((duty_percent > 0U) && (pulse == 0U))
  {
    pulse = 1U;
  }
  if ((duty_percent < 100U) && (pulse >= FAN_PWM_PERIOD))
  {
    pulse = FAN_PWM_PERIOD - 1U;
  }

  __HAL_TIM_SET_COMPARE(&htim4, FAN_TIM_CHANNEL, pulse);
}

uint8_t BSP_Fan_GetDutyPercent(void)
{
  return s_duty_percent;
}
