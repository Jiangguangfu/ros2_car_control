#include "bsp_buzzer.h"
#include "main.h"
#include "cmsis_os2.h"

#include <stdbool.h>

extern TIM_HandleTypeDef htim2;

/* TIM2: PSC=95, ARR=369 → 约 2.7kHz；占空比 30% */
#define BUZZER_PWM_TICKS     370U
#define BUZZER_PWM_DUTY_PCT   30U
#define BUZZER_PWM_PULSE     ((BUZZER_PWM_TICKS * BUZZER_PWM_DUTY_PCT) / 100U)
#define BUZZER_ON_MS         200U
#define BUZZER_OFF_MS        200U

static volatile uint8_t s_beeps_left;
static volatile bool s_on;
static uint32_t s_phase_ms;
static bool s_pwm_started;

static void Buzzer_PwmOn(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, BUZZER_PWM_PULSE);
  if (!s_pwm_started)
  {
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    s_pwm_started = true;
  }
}

static void Buzzer_PwmOff(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
}

void BSP_Buzzer_Init(void)
{
  s_beeps_left = 0U;
  s_on = false;
  s_phase_ms = 0U;
  Buzzer_PwmOff();
  if (!s_pwm_started)
  {
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    s_pwm_started = true;
  }
}

void BSP_Buzzer_Beep(uint32_t duration_ms)
{
  if (duration_ms == 0U)
  {
    return;
  }

  s_beeps_left = 0U;
  s_on = false;
  s_phase_ms = 0U;
  Buzzer_PwmOn();
  osDelay(duration_ms);
  Buzzer_PwmOff();
}

void BSP_Buzzer_Stop(void)
{
  Buzzer_PwmOff();
  s_beeps_left = 0U;
  s_on = false;
  s_phase_ms = 0U;
}

void BSP_Buzzer_PlayBeeps(uint8_t count)
{
  if (count == 0U)
  {
    BSP_Buzzer_Stop();
    return;
  }

  s_beeps_left = count;
  s_phase_ms = 0U;
  s_on = true;
  Buzzer_PwmOn();
}

void BSP_Buzzer_Process(uint32_t elapsed_ms)
{
  if ((s_beeps_left == 0U) && (!s_on))
  {
    return;
  }

  s_phase_ms += elapsed_ms;

  if (s_on)
  {
    if (s_phase_ms >= BUZZER_ON_MS)
    {
      Buzzer_PwmOff();
      s_on = false;
      s_phase_ms = 0U;
      if (s_beeps_left > 0U)
      {
        s_beeps_left--;
      }
    }
  }
  else if (s_beeps_left > 0U)
  {
    if (s_phase_ms >= BUZZER_OFF_MS)
    {
      Buzzer_PwmOn();
      s_on = true;
      s_phase_ms = 0U;
    }
  }
}
