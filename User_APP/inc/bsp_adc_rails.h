/**
 ******************************************************************************
 * @file    bsp_adc_rails.h
 * @brief   电源轨输出电压 / 电流 ADC 监测（DMA 扫描）
 *
 * MCU 引脚与 CubeMX Rank（STM32U375）：
 *   PA0 IN3 Rank1 — ADC_M_24V_O1    INA180A2 × 10 mΩ 电流（24V PGOOD，无电压分压）
 *   PA1 IN4 Rank2 — ADC_M_19V_O1    INA180A2 × 10 mΩ 电流（19V PGOOD，无电压分压）
 *   PA2 IN5 Rank3 — ADC_M_12V_O1    INA180A2 × 10 mΩ 电流（仅遥测）
 *   PA3 IN6 Rank4 — ADC_M_5V        INA180A2 × 10 mΩ 电流
 *   PA6 IN9 Rank5 — ADC_M_7.5V_O1   INA180A2 × 10 mΩ 电流（仅遥测）
 *   PA7 IN10 Rank6 — +7.5V_OUT3_V   分压 40k/10k（6.5V PGOOD）
 *   PB0 IN13 Rank7 — +12V_OUT2_V    分压 120k/20k（12V PGOOD）
 * 19V/24V 无电压采样，rail_mv 保持 0。
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
  uint32_t rail_mv[PWR_RAIL_COUNT];                /** 12V/6.5V 输出电压 mV；19V/24V/5V 无分压，为 0。 */
  uint32_t rail_ma[PWR_RAIL_COUNT];                /** 各轨输出电流，mA（INA180A2 × 10 mΩ）。 */
  uint16_t channel_raw[BSP_ADC_CHANNEL_COUNT];     /** DMA 半字原始采样（Rank 1..9，每通道一个 uint16）。 PA0,PA1,PA2,PA3,PA6,PA7,PB0,PB1,PB2*/
} adc_rails_status_t;

bool     BSP_AdcRails_Init(void);
bool     BSP_AdcRails_IsReady(void);
/** 从 DMA 缓冲刷新 rail_mv / rail_ma / channel_raw。 */
void     BSP_AdcRails_Update(void);
const adc_rails_status_t *BSP_AdcRails_GetStatus(void);
uint32_t BSP_AdcRails_GetRailMv(pwr_rail_id_t rail);
uint32_t BSP_AdcRails_GetRailMa(pwr_rail_id_t rail);
bool     BSP_AdcRails_IsRailGood(pwr_rail_id_t rail);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_RAILS_H */
