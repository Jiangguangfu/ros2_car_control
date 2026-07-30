#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 开机/提示音：duration_ms 期间输出 TIM2_CH3 PWM，然后关闭 */
void BSP_Buzzer_Beep(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUZZER_H */
