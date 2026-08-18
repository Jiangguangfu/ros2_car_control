/**
 ******************************************************************************
 * @file    bsp_adc_rails.h
 * @brief   电源轨输出电压 / 电流 ADC 监测（DMA 扫描）
 *
 * MCU 引脚与 CubeMX Rank（以实测校正 12V/24V）：
 *   PA0 IN3 Rank1 — ADC_M_12V_O1   电压分压 100k/30k
 *   PA1 IN4 Rank2 — ADC_M_19V_O1   电压分压 100k/20k
 *   PA2 IN5 Rank3 — ADC_M_24V_O1   电压分压 200k/20k
 *   PA3 IN6 Rank4 — ADC_M_5V       INA180A2 电流（非电压）
 *   PA6 IN9 Rank5 — 丝印 ADC_M_7.5V_O1，实测 raw=0，不是 6.5V
 *   PB2 IN15 Rank9 — 6.5V 实测 raw≈2046，分压按 4/1 标定
 ******************************************************************************
 */
#ifndef BSP_ADC_RAILS_H
#define BSP_ADC_RAILS_H

#include <stdint.h>
#include <stdbool.h>

#include "bsp_power_rails.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_ADC_CHANNEL_COUNT  9U

/** ADC 电源轨快照；由 BSP_AdcRails_Update() 刷新，只读访问请用 GetStatus()。 */
typedef struct
{
  bool ready;
  uint32_t rail_mv[PWR_RAIL_COUNT];                /** 各电源轨输出电压，mV；索引同 pwr_rail_id_t。5V 无电压分压，保持 0。 */
  uint32_t i5v_ma;                                 /** +5V_OUT3 负载电流，mA（INA180A2 × 10 mΩ）。 */
  uint16_t channel_raw[BSP_ADC_CHANNEL_COUNT];     /** DMA 半字原始采样（Rank 1..9，每通道一个 uint16）。 */
} adc_rails_status_t;

bool     BSP_AdcRails_Init(void);
bool     BSP_AdcRails_IsReady(void);
/** 从 DMA 缓冲刷新 rail_mv / i5v_ma / channel_raw。 */
void     BSP_AdcRails_Update(void);
const adc_rails_status_t *BSP_AdcRails_GetStatus(void);
uint32_t BSP_AdcRails_GetRailMv(pwr_rail_id_t rail);
bool     BSP_AdcRails_IsRailGood(pwr_rail_id_t rail);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_RAILS_H */
