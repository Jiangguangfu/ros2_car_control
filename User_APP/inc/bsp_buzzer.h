#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** TIM2_CH3 PWM 启动（CCR=0，静音）。 */
void BSP_Buzzer_Init(void);

/** 阻塞响一声，duration_ms 后关闭。 */
void BSP_Buzzer_Beep(uint32_t duration_ms);

/** 非阻塞：响 count 声（开/关各 200 ms）。重复调用会从头播。 */
void BSP_Buzzer_PlayBeeps(uint8_t count);

/** 立刻静音并取消未完成的蜂鸣序列。 */
void BSP_Buzzer_Stop(void);

/** 周期调用：报警调度 + 推进非阻塞蜂鸣序列。elapsed_ms 为距上次调用的间隔。 */
void Buzzer_AlarmProcess(uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUZZER_H */
