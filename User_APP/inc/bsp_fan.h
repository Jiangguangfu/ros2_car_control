/**
 ******************************************************************************
 * @file    bsp_fan.h
 * @brief   System fan PWM control (TIM4_CH3 / PB8).
 ******************************************************************************
 */
#ifndef BSP_FAN_H
#define BSP_FAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 1：上电后一直全速，不按温度/充电调速。改回 0 即恢复原逻辑。 */
#ifndef FAN_FORCE_FULL_SPEED
#define FAN_FORCE_FULL_SPEED  1
#endif

void BSP_Fan_Init(void);
/** duty_percent: 0..100 */
void BSP_Fan_SetDutyPercent(uint8_t duty_percent);
uint8_t BSP_Fan_GetDutyPercent(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FAN_H */
