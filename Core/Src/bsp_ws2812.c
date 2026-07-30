#include "bsp_ws2812.h"
#include "main.h"

/* 单颗状态灯；系统时钟 96MHz，用 DWT 周期计数做 WS2812 时序 */
#define WS2812_LED_COUNT   1U
#define WS2812_CPU_HZ      96000000U
#define WS2812_T0H_NS      350U
#define WS2812_T1H_NS      700U
#define WS2812_TBIT_NS     1250U
#define WS2812_RESET_US    80U

#define NS_TO_CYCLES(ns)   (((uint32_t)(ns) * (WS2812_CPU_HZ / 1000000U) + 999U) / 1000U)

static void ws2812_delay_cycles(uint32_t cycles)
{
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) {
  }
}

static void ws2812_enable_cycle_counter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void ws2812_send_bit(uint8_t bit)
{
  const uint32_t t0h = NS_TO_CYCLES(WS2812_T0H_NS);
  const uint32_t t1h = NS_TO_CYCLES(WS2812_T1H_NS);
  const uint32_t tbit = NS_TO_CYCLES(WS2812_TBIT_NS);
  uint32_t th = bit ? t1h : t0h;
  uint32_t start;

  start = DWT->CYCCNT;
  WS2812_DATA_GPIO_Port->BSRR = WS2812_DATA_Pin;
  while ((DWT->CYCCNT - start) < th) {
  }
  WS2812_DATA_GPIO_Port->BRR = WS2812_DATA_Pin;
  while ((DWT->CYCCNT - start) < tbit) {
  }
}

static void ws2812_send_byte(uint8_t value)
{
  for (int i = 7; i >= 0; i--) {
    ws2812_send_bit((uint8_t)((value >> i) & 0x01U));
  }
}

void BSP_WS2812_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();

  gpio.Pin = WS2812_DATA_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(WS2812_DATA_GPIO_Port, &gpio);
  HAL_GPIO_WritePin(WS2812_DATA_GPIO_Port, WS2812_DATA_Pin, GPIO_PIN_RESET);

  ws2812_enable_cycle_counter();
}

void BSP_WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  /* WS2812 线序：G-R-B */
  for (uint32_t i = 0; i < WS2812_LED_COUNT; i++) {
    (void)i;
    ws2812_send_byte(g);
    ws2812_send_byte(r);
    ws2812_send_byte(b);
  }

  HAL_GPIO_WritePin(WS2812_DATA_GPIO_Port, WS2812_DATA_Pin, GPIO_PIN_RESET);
  /* 复位低电平 */
  ws2812_delay_cycles(NS_TO_CYCLES(WS2812_RESET_US * 1000U));

  if (primask == 0U) {
    __enable_irq();
  }
}

void BSP_WS2812_Off(void)
{
  BSP_WS2812_SetColor(0, 0, 0);
}
