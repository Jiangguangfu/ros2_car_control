/**
 ******************************************************************************
 * @file    bq76942.h
 * @brief   BQ76942 I2C helpers: temperature, FET/DSG enable for 24V output.
 *
 * Schematic (BQ76942PBR):
 *   TS1/TS2 — NTC; TS3 — SW2
 *   PC13 PWR_24V_BYPASS_EN + BQ_DFETOFF low + FET_ENABLE + ALL_FETS_ON → 24V out
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

/* Direct commands */
#define BQ76942_CMD_BATTERY_STATUS        0x12U
#define BQ76942_CMD_FET_STATUS            0x7FU
#define BQ76942_CMD_INT_TEMP              0x68U
#define BQ76942_CMD_TS1_TEMP              0x70U
#define BQ76942_CMD_TS2_TEMP              0x72U
#define BQ76942_CMD_TS3_TEMP              0x74U

/* Subcommand mailbox */
#define BQ76942_REG_CMD_LOW               0x3EU
#define BQ76942_REG_CMD_HIGH              0x3FU
#define BQ76942_REG_DATA_START            0x40U

/* Subcommands */
#define BQ76942_SUBCMD_FET_ENABLE         0x0022U /* toggle Manufacturing[FET_EN] */
#define BQ76942_SUBCMD_ALL_FETS_ON        0x0096U
#define BQ76942_SUBCMD_ALL_FETS_OFF       0x0095U
#define BQ76942_SUBCMD_MFG_STATUS         0x0057U

/* 0x0057 Manufacturing Status bits */
#define BQ76942_MFG_FET_EN                (1U << 4)

/* 0x7F FET Status bits (common BQ769x2 mapping) */
#define BQ76942_FETSTAT_CHG_FET           (1U << 0)
#define BQ76942_FETSTAT_PCHG_FET          (1U << 1)
#define BQ76942_FETSTAT_DSG_FET           (1U << 2)
#define BQ76942_FETSTAT_PDSG_FET          (1U << 3)

typedef struct
{
  int16_t int_temp_0p1k;
  int16_t ts1_temp_0p1k;
  int16_t ts2_temp_0p1k;
  int16_t int_temp_c_x10;
  int16_t ts1_temp_c_x10;
  int16_t ts2_temp_c_x10;
  int16_t ts3_adcin_mv;
  bool valid;
} bq76942_temp_t;

bool BQ76942_IsReady(I2C_HandleTypeDef *hi2c);
bool BQ76942_ReadTempRaw(I2C_HandleTypeDef *hi2c, uint8_t cmd, int16_t *raw);
bool BQ76942_ReadTemperatures(I2C_HandleTypeDef *hi2c, bq76942_temp_t *out);

bool BQ76942_SubCommandWrite(I2C_HandleTypeDef *hi2c, uint16_t subcmd);
bool BQ76942_SubCommandReadU16(I2C_HandleTypeDef *hi2c, uint16_t subcmd, uint16_t *value);
bool BQ76942_ReadFetStatus(I2C_HandleTypeDef *hi2c, uint8_t *fet_status);

/**
 * Release MCU DFETOFF/CFETOFF, exit FET Test mode if needed, ALL_FETS_ON.
 * Required before DSG path can supply pack 24 V output.
 */
bool BQ76942_EnableDischargePath(I2C_HandleTypeDef *hi2c);

static inline int16_t BQ76942_Temp0p1KToCx10(int16_t temp_0p1k)
{
  return (int16_t)(temp_0p1k - 2732);
}

#ifdef __cplusplus
}
#endif

#endif /* BQ76942_H */
