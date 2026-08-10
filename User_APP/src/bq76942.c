/**
 ******************************************************************************
 * @file    bq76942.c
 * @brief   BQ76942 temperature, cells, FET, passive balance, measurements.
 ******************************************************************************
 */
#include "bq76942.h"
#include "main.h"
#include "cmsis_os2.h"
#include <string.h>

#define BQ76942_I2C_TIMEOUT_MS            50U
#define BQ76942_SUBCMD_WAIT_MS            2U

static uint8_t Bq76942_DmChecksum(const uint8_t *data, uint8_t len)
{
  uint8_t sum = 0U;
  uint8_t i;

  for (i = 0U; i < len; i++)
  {
    sum = (uint8_t)(sum + data[i]);
  }

  return (uint8_t)(~sum);
}

static bool BQ76942_ReadU16(I2C_HandleTypeDef *hi2c, uint8_t cmd, uint16_t *value)
{
  return BQ76942_ReadDirectU16(hi2c, cmd, value);
}

static bool BQ76942_WriteU16(I2C_HandleTypeDef *hi2c, uint8_t cmd, uint16_t value)
{
  uint8_t buf[2];

  if (hi2c == NULL)
  {
    return false;
  }

  buf[0] = (uint8_t)(value & 0xFFU);
  buf[1] = (uint8_t)((value >> 8) & 0xFFU);

  return (HAL_I2C_Mem_Write(hi2c, BQ76942_I2C_ADDR_HAL, cmd, I2C_MEMADD_SIZE_8BIT,
                            buf, sizeof(buf), BQ76942_I2C_TIMEOUT_MS) == HAL_OK);
}

bool BQ76942_IsReady(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return false;
  }

  return (HAL_I2C_IsDeviceReady(hi2c, BQ76942_I2C_ADDR_HAL, 3U, BQ76942_I2C_TIMEOUT_MS) == HAL_OK);
}

bool BQ76942_ReadDirectU16(I2C_HandleTypeDef *hi2c, uint8_t cmd, uint16_t *raw)
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

  *raw = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
  return true;
}

bool BQ76942_ReadDirectS16(I2C_HandleTypeDef *hi2c, uint8_t cmd, int16_t *raw)
{
  uint16_t u16;

  if (!BQ76942_ReadDirectU16(hi2c, cmd, &u16))
  {
    return false;
  }

  *raw = (int16_t)u16;
  return true;
}

bool BQ76942_ReadTempRaw(I2C_HandleTypeDef *hi2c, uint8_t cmd, int16_t *raw)
{
  return BQ76942_ReadDirectS16(hi2c, cmd, raw);
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

bool BQ76942_DataMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t addr,
                             const uint8_t *data, uint8_t len)
{
  uint8_t block[34];
  uint8_t meta[2];

  if ((hi2c == NULL) || (data == NULL) || (len == 0U) || (len > 32U))
  {
    return false;
  }

  block[0] = (uint8_t)(addr & 0xFFU);
  block[1] = (uint8_t)((addr >> 8) & 0xFFU);
  for (uint8_t i = 0U; i < len; i++)
  {
    block[2U + i] = data[i];
  }

  if (HAL_I2C_Mem_Write(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_CMD_LOW,
                        I2C_MEMADD_SIZE_8BIT, block, (uint16_t)(len + 2U),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  meta[0] = Bq76942_DmChecksum(block, (uint8_t)(len + 2U));
  meta[1] = (uint8_t)(len + 4U); /* addr(2) + data + 0x3E/0x3F/0x60/0x61 */
  if (HAL_I2C_Mem_Write(hi2c, BQ76942_I2C_ADDR_HAL, 0x60U,
                        I2C_MEMADD_SIZE_8BIT, meta, sizeof(meta),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  osDelay(5);
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

bool BQ76942_SubCommandRead(I2C_HandleTypeDef *hi2c, uint16_t subcmd,
                            uint8_t *data, uint8_t len)
{
  if ((hi2c == NULL) || (data == NULL) || (len == 0U) || (len > 32U))
  {
    return false;
  }

  if (!BQ76942_SubCommandWrite(hi2c, subcmd))
  {
    return false;
  }

  return (HAL_I2C_Mem_Read(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_DATA_START,
                           I2C_MEMADD_SIZE_8BIT, data, len,
                           BQ76942_I2C_TIMEOUT_MS) == HAL_OK);
}

bool BQ76942_SubCommandReadU16(I2C_HandleTypeDef *hi2c, uint16_t subcmd, uint16_t *value)
{
  uint8_t buf[2];

  if (value == NULL)
  {
    return false;
  }

  if (!BQ76942_SubCommandRead(hi2c, subcmd, buf, sizeof(buf)))
  {
    return false;
  }

  *value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
  return true;
}

static void BQ76942_FloatToLeBytes(float value, uint8_t out[4])
{
  union
  {
    float f;
    uint8_t b[4];
  } u;

  u.f = value;
  (void)memcpy(out, u.b, sizeof(u.b));
}

bool BQ76942_WriteVdivOffset(I2C_HandleTypeDef *hi2c, int16_t offset_userv)
{
  uint8_t val[2];

  val[0] = (uint8_t)((uint16_t)offset_userv & 0xFFU);
  val[1] = (uint8_t)(((uint16_t)offset_userv >> 8) & 0xFFU);

  return BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_Vdiv_OFFSET, val, sizeof(val));
}

bool BQ76942_WriteCcGain(I2C_HandleTypeDef *hi2c, float cc_gain)
{
  uint8_t cc_bytes[4];
  uint8_t cap_bytes[4];
  float capacity_gain;

  if (hi2c == NULL)
  {
    return false;
  }

  BQ76942_FloatToLeBytes(cc_gain, cc_bytes);
  capacity_gain = cc_gain * BQ76942_CAPACITY_GAIN_FACTOR;
  BQ76942_FloatToLeBytes(capacity_gain, cap_bytes);

  if (!BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_CC_GAIN, cc_bytes, sizeof(cc_bytes)))
  {
    return false;
  }

  return BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_CAPACITY_GAIN, cap_bytes, sizeof(cap_bytes));
}

bool BQ76942_WriteScdDelay(I2C_HandleTypeDef *hi2c, uint8_t delay_code)
{
  if ((hi2c == NULL) || (delay_code == 0U))
  {
    return false;
  }

  return BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_SCD_DELAY, &delay_code, 1U);
}

bool BQ76942_InitCalibration(I2C_HandleTypeDef *hi2c)
{
  float cc_gain;
  bool ok;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_IsReady(hi2c))
  {
    return false;
  }

  cc_gain = BQ76942_CC_GAIN_RSENSE_FACTOR / BQ76942_SENSE_RESISTOR_MOHM;
  cc_gain /= BQ76942_CC_GAIN_MEASURED_RATIO;

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_CONFIG_UPDATE))
  {
    return false;
  }
  osDelay(10);

  ok = BQ76942_WriteVdivOffset(hi2c, (int16_t)BQ76942_Vdiv_OFFSET_VALUE);
  ok = ok && BQ76942_WriteCcGain(hi2c, cc_gain);
  ok = ok && BQ76942_WriteScdDelay(hi2c, BQ76942_SCD_DELAY);

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_CONFIG_UPDATE_EXIT))
  {
    ok = false;
  }
  osDelay(10);
  return ok;
}

bool BQ76942_ReadMeasurements(I2C_HandleTypeDef *hi2c, bq76942_meas_t *out)
{
  uint16_t raw_u16;
  int16_t raw_s16;
  uint16_t vmin = 0xFFFFU;
  uint16_t vmax = 0U;
  uint8_t i;

  if (out == NULL)
  {
    return false;
  }

  out->valid = false;

  for (i = 0U; i < BQ76942_CELL_COUNT; i++)
  {
    if (!BQ76942_ReadDirectU16(hi2c, (uint8_t)(BQ76942_CMD_CELL1_VOLTAGE + (2U * i)), &raw_u16))
    {
      return false;
    }

    out->cell_mv[i] = raw_u16;
    if (raw_u16 < vmin)
    {
      vmin = raw_u16;
    }
    if (raw_u16 > vmax)
    {
      vmax = raw_u16;
    }
  }

  if (!BQ76942_ReadDirectU16(hi2c, BQ76942_CMD_STACK_VOLTAGE, &raw_u16))
  {
    return false;
  }
  out->pack_mv = BQ76942_UserVToMv(raw_u16);

  if (!BQ76942_ReadDirectU16(hi2c, BQ76942_CMD_PACK_VOLTAGE, &raw_u16))
  {
    return false;
  }
  out->output_mv = BQ76942_UserVToMv(raw_u16);

  if (!BQ76942_ReadDirectS16(hi2c, BQ76942_CMD_CC2_CURRENT, &raw_s16))
  {
    return false;
  }
  out->current_ma = raw_s16;

  /* CC3: averaged CC2 samples via DASTATUS5 bytes 20–21. */
  {
    uint8_t dastatus5[BQ76942_DASTATUS5_CC3_OFFSET + 2U];

    if (!BQ76942_SubCommandRead(hi2c, BQ76942_SUBCMD_DASTATUS5,
                                dastatus5, (uint8_t)sizeof(dastatus5)))
    {
      return false;
    }

    out->current_cc3_ma = (int16_t)((uint16_t)dastatus5[BQ76942_DASTATUS5_CC3_OFFSET] |
                                    ((uint16_t)dastatus5[BQ76942_DASTATUS5_CC3_OFFSET + 1U] << 8));
  }

  out->vcell_min_mv = vmin;
  out->vcell_max_mv = vmax;
  out->valid = true;
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

static bool BQ76942_EnsureFetsEnabled(I2C_HandleTypeDef *hi2c)
{
  uint16_t mfg_status = 0U;

  if (hi2c == NULL)
  {
    return false;
  }

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

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_ALL_FETS_ON))
  {
    return false;
  }

  osDelay(10);
  return true;
}

bool BQ76942_EnableDischargePath(I2C_HandleTypeDef *hi2c)
{
  uint8_t fet_status = 0U;

  if (hi2c == NULL)
  {
    return false;
  }

  /* 24V bypass: thermal_manager / boot sequence owns GPIO. */
  /* Release host DFETOFF for pack discharge; CFETOFF owned by charge_path. */
  HAL_GPIO_WritePin(BQ_DFETOFF_GPIO_Port, BQ_DFETOFF_Pin, GPIO_PIN_RESET);

  if (!BQ76942_EnsureFetsEnabled(hi2c))
  {
    return false;
  }

  if (!BQ76942_ReadFetStatus(hi2c, &fet_status))
  {
    return false;
  }

  /* Success if DSG FET driver reports on (pack discharge / 24V path). */
  return ((fet_status & BQ76942_FETSTAT_DSG_FET) != 0U);
}

bool BQ76942_EnableChargePath(I2C_HandleTypeDef *hi2c)
{
  uint8_t fet_status = 0U;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_EnsureFetsEnabled(hi2c))
  {
    return false;
  }

  if (!BQ76942_ReadFetStatus(hi2c, &fet_status))
  {
    return false;
  }

  return ((fet_status & BQ76942_FETSTAT_CHG_FET) != 0U);
}

bool BQ76942_ReadCellVoltages(I2C_HandleTypeDef *hi2c, uint8_t cell_count,
                              bq76942_cells_t *out)
{
  uint8_t i;

  if ((hi2c == NULL) || (out == NULL) || (cell_count == 0U) ||
      (cell_count > BQ76942_MAX_CELLS))
  {
    return false;
  }

  out->valid = false;
  out->cell_count = cell_count;

  for (i = 0U; i < cell_count; i++)
  {
    if (!BQ76942_ReadU16(hi2c, BQ76942_CellVoltageCmd(i), &out->cell_mv[i]))
    {
      return false;
    }
  }

  if (!BQ76942_ReadU16(hi2c, BQ76942_CMD_STACK_VOLTAGE, &out->stack_mv))
  {
    return false;
  }

  out->valid = true;
  return true;
}

bool BQ76942_ReadPackCurrent(I2C_HandleTypeDef *hi2c, int16_t *current_ma)
{
  int16_t raw;

  if ((hi2c == NULL) || (current_ma == NULL))
  {
    return false;
  }

  if (!BQ76942_ReadDirectS16(hi2c, BQ76942_CMD_CC2_CURRENT, &raw))
  {
    return false;
  }

  *current_ma = raw;
  return true;
}

bool BQ76942_ReadBatteryStatus(I2C_HandleTypeDef *hi2c, uint16_t *status)
{
  if ((hi2c == NULL) || (status == NULL))
  {
    return false;
  }

  return BQ76942_ReadU16(hi2c, BQ76942_CMD_BATTERY_STATUS, status);
}

bool BQ76942_ReadSafetyStatus(I2C_HandleTypeDef *hi2c, bool *protect_active)
{
  uint8_t sa;
  uint8_t sb;
  uint8_t sc;

  if ((hi2c == NULL) || (protect_active == NULL))
  {
    return false;
  }

  *protect_active = false;

  if (HAL_I2C_Mem_Read(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_CMD_SAFETY_STATUS_A,
                       I2C_MEMADD_SIZE_8BIT, &sa, 1U,
                       BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  if (HAL_I2C_Mem_Read(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_CMD_SAFETY_STATUS_B,
                       I2C_MEMADD_SIZE_8BIT, &sb, 1U,
                       BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  if (HAL_I2C_Mem_Read(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_CMD_SAFETY_STATUS_C,
                       I2C_MEMADD_SIZE_8BIT, &sc, 1U,
                       BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  *protect_active = ((sa != 0U) || (sb != 0U) || (sc != 0U));
  return true;
}

bool BQ76942_SetBalanceMask(I2C_HandleTypeDef *hi2c, uint16_t mask)
{
  if (hi2c == NULL)
  {
    return false;
  }

  return BQ76942_WriteU16(hi2c, BQ76942_CMD_CB_ACTIVE_CELLS, mask);
}

bool BQ76942_ReadBalanceMask(I2C_HandleTypeDef *hi2c, uint16_t *mask)
{
  if ((hi2c == NULL) || (mask == NULL))
  {
    return false;
  }

  return BQ76942_ReadU16(hi2c, BQ76942_CMD_CB_ACTIVE_CELLS, mask);
}
