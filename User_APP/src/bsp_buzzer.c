#include "bsp_buzzer.h"
#include "main.h"
#include "cmsis_os2.h"

extern TIM_HandleTypeDef htim2;

/* TIM2: PSC=95, ARR=369 → 约 2.7kHz；占空比约 50% */
#define BUZZER_PWM_PULSE  185U

void BSP_Buzzer_Beep(uint32_t duration_ms)
{
  if (duration_ms == 0U) {
    return;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, BUZZER_PWM_PULSE);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  osDelay(duration_ms);
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
}
