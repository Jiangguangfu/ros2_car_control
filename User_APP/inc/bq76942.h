/**
 ******************************************************************************
 * @file    bq76942.h
 * @brief   Minimal BQ76942 I2C helpers for board temperature / TS3 button.
 *
 * Schematic (BQ76942PBR):
 *   TS1 — NTC 10K-103F3950FM to VSS (+100nF)
 *   TS2 — NTC 10K-103F3950FM to VSS (+100nF)
 *   TS3 — SW2 push-button path (not a thermistor); read as ADCIN mV
 ******************************************************************************
 */
#ifndef BQ76942_H
#define BQ76942_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32u3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Datasheet / TRM: 8-bit write address 0x10 (7-bit 0x08). HAL uses 7-bit << 1. */
#define BQ76942_I2C_ADDR_HAL              0x10U

/* Direct commands — Int/TS1/TS2: 0.1 K; TS3 ADCIN mode: mV */
#define BQ76942_CMD_INT_TEMP              0x68U /*内部芯片温度*/
#define BQ76942_CMD_TS1_TEMP              0x70U /*TS1引脚热敏电阻*/
#define BQ76942_CMD_TS2_TEMP              0x72U /*TS2引脚热敏电阻*/
#define BQ76942_CMD_TS3_TEMP              0x74U /*TS3引脚按钮*/

typedef struct
{
  int16_t int_temp_0p1k;   /* die temperature, 0.1 K */
  int16_t ts1_temp_0p1k;   /* cell/pack NTC on TS1, 0.1 K */
  int16_t ts2_temp_0p1k;   /* cell/pack NTC on TS2, 0.1 K */
  int16_t int_temp_c_x10;  /* Celsius * 10 */
  int16_t ts1_temp_c_x10;
  int16_t ts2_temp_c_x10;
  /* TS3 = SW2: when pin configured as ADCIN, command returns mV */
  int16_t ts3_adcin_mv;
  bool valid;
} bq76942_temp_t;

bool BQ76942_IsReady(I2C_HandleTypeDef *hi2c);
bool BQ76942_ReadTempRaw(I2C_HandleTypeDef *hi2c, uint8_t cmd, int16_t *raw);
bool BQ76942_ReadTemperatures(I2C_HandleTypeDef *hi2c, bq76942_temp_t *out);

/** Convert 0.1 K reading to Celsius * 10. */
static inline int16_t BQ76942_Temp0p1KToCx10(int16_t temp_0p1k)
{
  return (int16_t)(temp_0p1k - 2732);
}

#ifdef __cplusplus
}
#endif

#endif /* BQ76942_H */
