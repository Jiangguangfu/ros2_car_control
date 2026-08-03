/**
 ******************************************************************************
 * @file    bq76942.c
 * @brief   BQ76942 temperature (TS1/TS2) + TS3 button ADCIN read.
 ******************************************************************************
 */
#include "bq76942.h"

#define BQ76942_I2C_TIMEOUT_MS            50U

bool BQ76942_IsReady(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return false;
  }

  return (HAL_I2C_IsDeviceReady(hi2c, BQ76942_I2C_ADDR_HAL, 3U, BQ76942_I2C_TIMEOUT_MS) == HAL_OK);
}

bool BQ76942_ReadTempRaw(I2C_HandleTypeDef *hi2c, uint8_t cmd, int16_t *raw)
{
  uint8_t buf[2];

  if ((hi2c == NULL) || (raw == NULL))
  {
    return false;
  }

  if (HAL_I2C_Mem_Read(hi2c, BQ76942_I2C_ADDR_HAL, cmd, I2C_MEMADD_SIZE_8BIT,
                       buf, sizeof(buf), BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  *raw = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
  return true;
}

bool BQ76942_ReadTemperatures(I2C_HandleTypeDef *hi2c, bq76942_temp_t *out)
{
  int16_t raw;

  if (out == NULL)
  {
    return false;
  }

  out->valid = false;

  /* Internal die temperature */
  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_INT_TEMP, &raw))
  {
    return false;
  }
  out->int_temp_0p1k = raw;
  out->int_temp_c_x10 = BQ76942_Temp0p1KToCx10(raw);

  /* TS1 / TS2: NTC 10K-103F3950FM */
  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_TS1_TEMP, &raw))
  {
    return false;
  }
  out->ts1_temp_0p1k = raw;
  out->ts1_temp_c_x10 = BQ76942_Temp0p1KToCx10(raw);

  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_TS2_TEMP, &raw))
  {
    return false;
  }
  out->ts2_temp_0p1k = raw;
  out->ts2_temp_c_x10 = BQ76942_Temp0p1KToCx10(raw);

  /* TS3: SW2 — not a thermistor; keep raw as mV when pin is ADCIN */
  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_TS3_TEMP, &raw))
  {
    return false;
  }
  out->ts3_adcin_mv = raw;

  out->valid = true;
  return true;
}
