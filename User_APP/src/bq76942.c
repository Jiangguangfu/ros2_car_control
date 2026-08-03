/**
 ******************************************************************************
 * @file    bq76942.c
 * @brief   BQ76942 temperature + DSG/FET enable for 24V pack output.
 ******************************************************************************
 */
#include "bq76942.h"
#include "main.h"
#include "cmsis_os2.h"

#define BQ76942_I2C_TIMEOUT_MS            50U
#define BQ76942_SUBCMD_WAIT_MS            2U

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

  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_INT_TEMP, &raw))
  {
    return false;
  }
  out->int_temp_0p1k = raw;
  out->int_temp_c_x10 = BQ76942_Temp0p1KToCx10(raw);

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

  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_TS3_TEMP, &raw))
  {
    return false;
  }
  out->ts3_adcin_mv = raw;

  out->valid = true;
  return true;
}

bool BQ76942_SubCommandWrite(I2C_HandleTypeDef *hi2c, uint16_t subcmd)
{
  uint8_t buf[2];

  if (hi2c == NULL)
  {
    return false;
  }

  buf[0] = (uint8_t)(subcmd & 0xFFU);
  buf[1] = (uint8_t)((subcmd >> 8) & 0xFFU);

  if (HAL_I2C_Mem_Write(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_CMD_LOW,
                        I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  osDelay(BQ76942_SUBCMD_WAIT_MS);
  return true;
}

bool BQ76942_SubCommandReadU16(I2C_HandleTypeDef *hi2c, uint16_t subcmd, uint16_t *value)
{
  uint8_t buf[2];

  if ((hi2c == NULL) || (value == NULL))
  {
    return false;
  }

  if (!BQ76942_SubCommandWrite(hi2c, subcmd))
  {
    return false;
  }

  if (HAL_I2C_Mem_Read(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_DATA_START,
                       I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf),
                       BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  *value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
  return true;
}

bool BQ76942_ReadFetStatus(I2C_HandleTypeDef *hi2c, uint8_t *fet_status)
{
  if ((hi2c == NULL) || (fet_status == NULL))
  {
    return false;
  }

  return (HAL_I2C_Mem_Read(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_CMD_FET_STATUS,
                           I2C_MEMADD_SIZE_8BIT, fet_status, 1U,
                           BQ76942_I2C_TIMEOUT_MS) == HAL_OK);
}

bool BQ76942_EnableDischargePath(I2C_HandleTypeDef *hi2c)
{
  uint16_t mfg_status = 0U;
  uint8_t fet_status = 0U;

  if (hi2c == NULL)
  {
    return false;
  }

  /* PC13: 24V bypass control — board path enable (independent of BQ I2C). */
  HAL_GPIO_WritePin(PWR_24V_BYPASS_EN_GPIO_Port, PWR_24V_BYPASS_EN_Pin, GPIO_PIN_SET);

  /* Host must not force FETs off via pins (active-high inhibit on this board). */
  HAL_GPIO_WritePin(BQ_DFETOFF_GPIO_Port, BQ_DFETOFF_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BQ_CFETOFF_GPIO_Port, BQ_CFETOFF_Pin, GPIO_PIN_RESET);

  /* Default power-up is often FET Test mode (FET_EN=0). Toggle into normal. */
  if (!BQ76942_SubCommandReadU16(hi2c, BQ76942_SUBCMD_MFG_STATUS, &mfg_status))
  {
    return false;
  }

  if ((mfg_status & BQ76942_MFG_FET_EN) == 0U)
  {
    if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_FET_ENABLE))
    {
      return false;
    }
    osDelay(10);
  }

  /* Allow CHG/DSG drivers if no protection is blocking. */
  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_ALL_FETS_ON))
  {
    return false;
  }
  osDelay(10);

  if (!BQ76942_ReadFetStatus(hi2c, &fet_status))
  {
    return false;
  }

  /* Success if DSG FET driver reports on (pack discharge / 24V path). */
  return ((fet_status & BQ76942_FETSTAT_DSG_FET) != 0U);
}
