#ifndef BSP_WS2812_H
#define BSP_WS2812_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} bsp_rgb_t;

void BSP_WS2812_Init(void);

/* 设置第 0 颗灯颜色（GRB 协议），立即刷新 */
void BSP_WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b);

void BSP_WS2812_Off(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_WS2812_H */
