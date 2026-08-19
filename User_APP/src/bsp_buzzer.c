#include "bsp_buzzer.h"
#include "bsp_power_rails.h"
#include "cell_voltage_protect.h"
#include "main.h"
#include "cmsis_os2.h"

#include <stdbool.h>

extern TIM_HandleTypeDef htim2;

/* TIM2: PSC=95, ARR=369 → 约 2.7kHz；占空比 30% */
#define BUZZER_PWM_TICKS            370U
#define BUZZER_PWM_DUTY_PCT          30U
#define BUZZER_PWM_PULSE             ((BUZZER_PWM_TICKS * BUZZER_PWM_DUTY_PCT) / 100U)
#define BUZZER_ON_MS                 200U
#define BUZZER_OFF_MS                200U

#define BUZZER_OT_FAULT_BEEPS          3U
#define BUZZER_OT_LIMIT_BEEPS          2U
#define BUZZER_OT_PERIOD_MS         4000U
#define BUZZER_LOW_BATT_BEEPS          6U
#define BUZZER_LOW_BATT_PERIOD_MS   8000U

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

void Buzzer_AlarmProcess(uint32_t elapsed_ms)
{
  static uint32_t s_alarm_ms = 0U;
  static uint8_t s_last_beeps = 0U;
  uint8_t beeps = 0U;
  uint32_t period_ms = 0U;
  pwr_state_t th_state = BSP_PowerRails_GetThermalState();
  pwr_reason_t th_reason = BSP_PowerRails_GetThermalReason();

  /* 过温故障 3 声/4s > 过温 LIMIT 2 声/4s > 低电量 6 声/8s。 */
  if ((th_state == PWR_STATE_FAULT) && (th_reason == PWR_REASON_HOT))
  {
    beeps = BUZZER_OT_FAULT_BEEPS;
    period_ms = BUZZER_OT_PERIOD_MS;
  }
  else if ((th_state == PWR_STATE_LIMIT) && (th_reason == PWR_REASON_HOT))
  {
    beeps = BUZZER_OT_LIMIT_BEEPS;
    period_ms = BUZZER_OT_PERIOD_MS;
  }
  else if (CellVoltageProtect_IsLowVoltageWarn())
  {
    beeps = BUZZER_LOW_BATT_BEEPS;
    period_ms = BUZZER_LOW_BATT_PERIOD_MS;
  }

  if (beeps != 0U)
  {
    if ((s_alarm_ms == 0U) || (beeps != s_last_beeps))
    {
      BSP_Buzzer_PlayBeeps(beeps);
      s_last_beeps = beeps;
      s_alarm_ms = 0U;
    }

    s_alarm_ms += elapsed_ms;
    if (s_alarm_ms >= period_ms)
    {
      s_alarm_ms = 0U;
    }
  }
  else
  {
    s_alarm_ms = 0U;
    s_last_beeps = 0U;
  }

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
