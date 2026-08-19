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
#define BQ76942_BALANCE_SUBCMD_WAIT_MS    20U
#define BQ76942_FET_ON_WAIT_MS            10U
#define BQ76942_FET_ON_RETRY              20U

static osMutexId_t s_i2c_mutex;
static const osMutexAttr_t s_i2c_mutex_attr = {
  .name = "BqI2cMtx",
  .attr_bits = osMutexRecursive | osMutexPrioInherit,
};

void BQ76942_LockInit(void)
{
  if (s_i2c_mutex == NULL)
  {
    s_i2c_mutex = osMutexNew(&s_i2c_mutex_attr);
  }
}

bool BQ76942_I2cLock(void)
{
  if (s_i2c_mutex == NULL)
  {
    BQ76942_LockInit();
  }

  if (s_i2c_mutex == NULL)
  {
    return false;
  }

  return (osMutexAcquire(s_i2c_mutex, osWaitForever) == osOK);
}

void BQ76942_I2cUnlock(void)
{
  if (s_i2c_mutex != NULL)
  {
    (void)osMutexRelease(s_i2c_mutex);
  }
}

static HAL_StatusTypeDef BqI2cMemWrite(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                       uint16_t mem, uint16_t mem_size,
                                       uint8_t *data, uint16_t len,
                                       uint32_t timeout)
{
  HAL_StatusTypeDef st;

  if (!BQ76942_I2cLock())
  {
    return HAL_ERROR;
  }

  st = HAL_I2C_Mem_Write(hi2c, dev, mem, mem_size, data, len, timeout);
  BQ76942_I2cUnlock();
  return st;
}

static HAL_StatusTypeDef BqI2cMemRead(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                      uint16_t mem, uint16_t mem_size,
                                      uint8_t *data, uint16_t len,
                                      uint32_t timeout)
{
  HAL_StatusTypeDef st;

  if (!BQ76942_I2cLock())
  {
    return HAL_ERROR;
  }

  st = HAL_I2C_Mem_Read(hi2c, dev, mem, mem_size, data, len, timeout);
  BQ76942_I2cUnlock();
  return st;
}

static HAL_StatusTypeDef BqI2cIsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t dev,
                                            uint32_t trials, uint32_t timeout)
{
  HAL_StatusTypeDef st;

  if (!BQ76942_I2cLock())
  {
    return HAL_ERROR;
  }

  st = HAL_I2C_IsDeviceReady(hi2c, dev, trials, timeout);
  BQ76942_I2cUnlock();
  return st;
}

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

bool BQ76942_IsReady(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return false;
  }

  return (BqI2cIsDeviceReady(hi2c, BQ76942_I2C_ADDR_HAL, 3U, BQ76942_I2C_TIMEOUT_MS) == HAL_OK);
}

bool BQ76942_ReadDirectU16(I2C_HandleTypeDef *hi2c, uint8_t cmd, uint16_t *raw)
{
  uint8_t buf[2];

  if ((hi2c == NULL) || (raw == NULL))
  {
    return false;
  }

  if (BqI2cMemRead(hi2c, BQ76942_I2C_ADDR_HAL, cmd, I2C_MEMADD_SIZE_8BIT,
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
  bool ok = false;

  if (out == NULL)
  {
    return false;
  }

  out->valid = false;

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_INT_TEMP, &raw))
  {
    goto out;
  }
  out->int_temp_0p1k = raw;
  out->int_temp_c_x10 = BQ76942_Temp0p1KToCx10(raw);

  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_TS1_TEMP, &raw))
  {
    goto out;
  }
  out->ts1_temp_0p1k = raw;
  out->ts1_temp_c_x10 = BQ76942_Temp0p1KToCx10(raw);

  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_TS2_TEMP, &raw))
  {
    goto out;
  }
  out->ts2_temp_0p1k = raw;
  out->ts2_temp_c_x10 = BQ76942_Temp0p1KToCx10(raw);

  if (!BQ76942_ReadTempRaw(hi2c, BQ76942_CMD_TS3_TEMP, &raw))
  {
    goto out;
  }
  out->ts3_adcin_mv = raw;

  out->valid = true;
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_DataMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t addr,
                             const uint8_t *data, uint8_t len)
{
  uint8_t block[34];
  uint8_t meta[2];
  bool ok = false;

  if ((hi2c == NULL) || (data == NULL) || (len == 0U) || (len > 32U))
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  block[0] = (uint8_t)(addr & 0xFFU);
  block[1] = (uint8_t)((addr >> 8) & 0xFFU);
  for (uint8_t i = 0U; i < len; i++)
  {
    block[2U + i] = data[i];
  }

  if (BqI2cMemWrite(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_CMD_LOW,
                        I2C_MEMADD_SIZE_8BIT, block, (uint16_t)(len + 2U),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  meta[0] = Bq76942_DmChecksum(block, (uint8_t)(len + 2U));
  meta[1] = (uint8_t)(len + 4U); /* addr(2) + data + 0x3E/0x3F/0x60/0x61 */
  if (BqI2cMemWrite(hi2c, BQ76942_I2C_ADDR_HAL, 0x60U,
                        I2C_MEMADD_SIZE_8BIT, meta, sizeof(meta),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  osDelay(5);
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_DataMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t addr,
                            uint8_t *data, uint8_t len)
{
  uint8_t addr_buf[2];
  bool ok = false;

  if ((hi2c == NULL) || (data == NULL) || (len == 0U) || (len > 32U))
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  addr_buf[0] = (uint8_t)(addr & 0xFFU);
  addr_buf[1] = (uint8_t)((addr >> 8) & 0xFFU);

  if (BqI2cMemWrite(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_CMD_LOW,
                        I2C_MEMADD_SIZE_8BIT, addr_buf, sizeof(addr_buf),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  osDelay(BQ76942_SUBCMD_WAIT_MS);

  ok = (BqI2cMemRead(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_DATA_START,
                     I2C_MEMADD_SIZE_8BIT, data, len,
                     BQ76942_I2C_TIMEOUT_MS) == HAL_OK);

out:
  BQ76942_I2cUnlock();
  return ok;
}

static const uint16_t s_scd_threshold_mv[16] = {
  10U,  20U,  40U,  60U,  80U,  100U, 125U, 150U,
  175U, 200U, 250U, 300U, 350U, 400U, 450U, 500U
};

static uint16_t Bq76942_ThresholdCodeToMv(uint8_t code)
{
  return (uint16_t)((uint16_t)code * 2U);
}

static uint16_t Bq76942_CellThresholdCodeToMv(uint8_t code)
{
  /* TRM: Protections:CUV/COV threshold = code × 50.6 mV. */
  return (uint16_t)(((uint32_t)code * BQ76942_CELL_THRESHOLD_MV_FACTOR + 5U) / 10U);
}

static uint16_t Bq76942_ProtDelayMsX10(uint8_t code)
{
  if (code == 0U)
  {
    return 0U;
  }

  /* TRM: 3.3 ms × (2 + setting) → store as 0.1 ms. */
  return (uint16_t)(33U * (2U + (uint16_t)code));
}

static uint16_t Bq76942_ScdDelayUs(uint8_t code)
{
  if (code == 0U)
  {
    return 0U;
  }

  return (uint16_t)((code - 1U) * 15U);
}

static uint16_t Bq76942_MvToMa(uint16_t threshold_mv)
{
  if (BQ76942_SENSE_RESISTOR_MOHM <= 0.0f)
  {
    return 0U;
  }

  return (uint16_t)((float)threshold_mv / BQ76942_SENSE_RESISTOR_MOHM);
}

bool BQ76942_ReadProtectionConfig(I2C_HandleTypeDef *hi2c,
                                  bq76942_prot_cfg_t *out)
{
  uint8_t block[BQ76942_DM_PROT_BLOCK_LEN];
  uint8_t enabled_prot_a = 0U;
  uint8_t cuv_threshold_code = 0U;
  uint8_t cov_threshold_code = 0U;
  bool ok = false;

  if ((hi2c == NULL) || (out == NULL))
  {
    return false;
  }

  (void)memset(out, 0, sizeof(*out));

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_IsReady(hi2c))
  {
    goto out;
  }

  if (!BQ76942_DataMemoryRead(hi2c, BQ76942_DM_ENABLED_PROT_A,
                              &enabled_prot_a, 1U))
  {
    goto out;
  }

  if (!BQ76942_DataMemoryRead(hi2c, BQ76942_DM_CUV_THRESHOLD,
                              &cuv_threshold_code, 1U))
  {
    goto out;
  }

  if (!BQ76942_DataMemoryRead(hi2c, BQ76942_DM_COV_THRESHOLD,
                              &cov_threshold_code, 1U))
  {
    goto out;
  }

  if (!BQ76942_DataMemoryRead(hi2c, BQ76942_DM_OCC_THRESHOLD,
                              block, (uint8_t)sizeof(block)))
  {
    goto out;
  }

  out->enabled_prot_a = enabled_prot_a;
  out->occ_enabled = ((enabled_prot_a & BQ76942_PROT_A_OCC) != 0U);
  out->ocd1_enabled = ((enabled_prot_a & BQ76942_PROT_A_OCD1) != 0U);
  out->ocd2_enabled = ((enabled_prot_a & BQ76942_PROT_A_OCD2) != 0U);
  out->scd_enabled = ((enabled_prot_a & BQ76942_PROT_A_SCD) != 0U);
  out->cov_enabled = ((enabled_prot_a & BQ76942_PROT_A_COV) != 0U);
  out->cuv_enabled = ((enabled_prot_a & BQ76942_PROT_A_CUV) != 0U);

  out->cuv_threshold_code = cuv_threshold_code;
  out->cov_threshold_code = cov_threshold_code;
  out->cuv_threshold_mv = Bq76942_CellThresholdCodeToMv(cuv_threshold_code);
  out->cov_threshold_mv = Bq76942_CellThresholdCodeToMv(cov_threshold_code);

  out->occ_threshold_code = block[0];
  out->occ_delay_code = block[1];
  out->ocd1_threshold_code = block[2];
  out->ocd1_delay_code = block[3];
  out->ocd2_threshold_code = block[4];
  out->ocd2_delay_code = block[5];
  out->scd_threshold_code = block[6];
  out->scd_delay_code = block[7];

  out->occ_threshold_mv = Bq76942_ThresholdCodeToMv(out->occ_threshold_code);
  out->ocd1_threshold_mv = Bq76942_ThresholdCodeToMv(out->ocd1_threshold_code);
  out->ocd2_threshold_mv = Bq76942_ThresholdCodeToMv(out->ocd2_threshold_code);

  if (out->scd_threshold_code < (uint8_t)(sizeof(s_scd_threshold_mv) /
                                         sizeof(s_scd_threshold_mv[0])))
  {
    out->scd_threshold_mv = s_scd_threshold_mv[out->scd_threshold_code];
  }

  out->occ_delay_ms_x10 = Bq76942_ProtDelayMsX10(out->occ_delay_code);
  out->ocd1_delay_ms_x10 = Bq76942_ProtDelayMsX10(out->ocd1_delay_code);
  out->ocd2_delay_ms_x10 = Bq76942_ProtDelayMsX10(out->ocd2_delay_code);
  out->scd_delay_us = Bq76942_ScdDelayUs(out->scd_delay_code);

  out->occ_trip_ma = Bq76942_MvToMa(out->occ_threshold_mv);
  out->ocd1_trip_ma = Bq76942_MvToMa(out->ocd1_threshold_mv);
  out->ocd2_trip_ma = Bq76942_MvToMa(out->ocd2_threshold_mv);
  out->scd_trip_ma = Bq76942_MvToMa(out->scd_threshold_mv);

  out->valid = true;
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_SubCommandWrite(I2C_HandleTypeDef *hi2c, uint16_t subcmd)
{
  uint8_t buf[2];
  bool ok = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  buf[0] = (uint8_t)(subcmd & 0xFFU);
  buf[1] = (uint8_t)((subcmd >> 8) & 0xFFU);

  if (BqI2cMemWrite(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_CMD_LOW,
                        I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  osDelay(BQ76942_SUBCMD_WAIT_MS);
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_SubCommandRead(I2C_HandleTypeDef *hi2c, uint16_t subcmd,
                            uint8_t *data, uint8_t len)
{
  bool ok = false;

  if ((hi2c == NULL) || (data == NULL) || (len == 0U) || (len > 32U))
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_SubCommandWrite(hi2c, subcmd))
  {
    goto out;
  }

  ok = (BqI2cMemRead(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_DATA_START,
                     I2C_MEMADD_SIZE_8BIT, data, len,
                     BQ76942_I2C_TIMEOUT_MS) == HAL_OK);

out:
  BQ76942_I2cUnlock();
  return ok;
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

bool BQ76942_ApplyVdivOffset1V(I2C_HandleTypeDef *hi2c)
{
  bool ok = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_CONFIG_UPDATE))
  {
    goto out;
  }
  osDelay(10);

  if (!BQ76942_WriteVdivOffset(hi2c, (int16_t)BQ76942_Vdiv_OFFSET_VALUE))
  {
    (void)BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_CONFIG_UPDATE_EXIT);
    goto out;
  }

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_CONFIG_UPDATE_EXIT))
  {
    goto out;
  }

  osDelay(10);
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_WriteCcGain(I2C_HandleTypeDef *hi2c, float cc_gain)
{
  uint8_t cc_bytes[4];
  uint8_t cap_bytes[4];
  float capacity_gain;
  bool ok = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  BQ76942_FloatToLeBytes(cc_gain, cc_bytes);
  capacity_gain = cc_gain * BQ76942_CAPACITY_GAIN_FACTOR;
  BQ76942_FloatToLeBytes(capacity_gain, cap_bytes);

  if (!BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_CC_GAIN, cc_bytes, sizeof(cc_bytes)))
  {
    goto out;
  }

  ok = BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_CAPACITY_GAIN, cap_bytes, sizeof(cap_bytes));

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_WriteScdDelay(I2C_HandleTypeDef *hi2c, uint8_t delay_code)
{
  if ((hi2c == NULL) || (delay_code == 0U))
  {
    return false;
  }

  return BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_SCD_DELAY, &delay_code, 1U);
}

bool BQ76942_WriteProtectionConfig(I2C_HandleTypeDef *hi2c)
{
  uint8_t enabled_prot_a = BQ76942_ENABLED_PROT_A;
  uint8_t chg_fet_prot_a = BQ76942_CHG_FET_PROT_A;
  uint8_t dsg_fet_prot_a = BQ76942_DSG_FET_PROT_A;
  uint8_t cuv_threshold = BQ76942_CUV_THRESHOLD;
  uint8_t block[BQ76942_DM_PROT_BLOCK_LEN];
  bool ok;

  if (hi2c == NULL)
  {
    return false;
  }

  if ((BQ76942_OCC_THRESHOLD < 2U) || (BQ76942_OCC_DELAY == 0U) ||
      (BQ76942_OCD1_THRESHOLD < 2U) || (BQ76942_OCD1_DELAY == 0U) ||
      (BQ76942_OCD2_THRESHOLD < 2U) || (BQ76942_OCD2_DELAY == 0U) ||
      (BQ76942_SCD_DELAY == 0U))
  {
    return false;
  }

  block[0] = BQ76942_OCC_THRESHOLD;
  block[1] = BQ76942_OCC_DELAY;
  block[2] = BQ76942_OCD1_THRESHOLD;
  block[3] = BQ76942_OCD1_DELAY;
  block[4] = BQ76942_OCD2_THRESHOLD;
  block[5] = BQ76942_OCD2_DELAY;
  block[6] = BQ76942_SCD_THRESHOLD;
  block[7] = BQ76942_SCD_DELAY;

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  ok = BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_ENABLED_PROT_A,
                               &enabled_prot_a, 1U);
  ok = ok && BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_CHG_FET_PROT_A,
                                     &chg_fet_prot_a, 1U);
  ok = ok && BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_DSG_FET_PROT_A,
                                     &dsg_fet_prot_a, 1U);
  ok = ok && BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_CUV_THRESHOLD,
                                     &cuv_threshold, 1U);
  ok = ok && BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_OCC_THRESHOLD,
                                     block, (uint8_t)sizeof(block));
  BQ76942_I2cUnlock();
  return ok;
}

static bool BQ76942_WriteTs2Config(I2C_HandleTypeDef *hi2c, uint8_t config)
{
  return BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_TS2_CONFIG, &config, 1U);
}

static bool BQ76942_WriteVcellMode(I2C_HandleTypeDef *hi2c, uint16_t mode)
{
  uint8_t block[2];

  block[0] = (uint8_t)(mode & 0xFFU);
  block[1] = (uint8_t)((mode >> 8) & 0xFFU);
  return BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_VCELL_MODE, block, 2U);
}

static bool BQ76942_WriteFetPredischargeConfig(I2C_HandleTypeDef *hi2c);

bool BQ76942_InitCalibration(I2C_HandleTypeDef *hi2c)
{
  float cc_gain;
  bool ok = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_IsReady(hi2c))
  {
    goto out;
  }

  cc_gain = BQ76942_CC_GAIN_RSENSE_FACTOR / BQ76942_SENSE_RESISTOR_MOHM;
  cc_gain /= BQ76942_CC_GAIN_MEASURED_RATIO;

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_CONFIG_UPDATE))
  {
    goto out;
  }
  osDelay(10);

  ok = BQ76942_WriteVdivOffset(hi2c, (int16_t)BQ76942_Vdiv_OFFSET_VALUE);
  ok = ok && BQ76942_WriteCcGain(hi2c, cc_gain);
  ok = ok && BQ76942_WriteVcellMode(hi2c, BQ76942_VCELL_MODE);
  ok = ok && BQ76942_WriteProtectionConfig(hi2c);
  ok = ok && BQ76942_WriteTs2Config(hi2c, BQ76942_TS2_CONFIG_REPORT_ONLY);
  ok = ok && BQ76942_WriteFetPredischargeConfig(hi2c);

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_CONFIG_UPDATE_EXIT))
  {
    ok = false;
  }
  osDelay(10);

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_ReadMeasurements(I2C_HandleTypeDef *hi2c, bq76942_meas_t *out)
{
  uint16_t raw_u16;
  int16_t raw_s16;
  uint16_t vmin = 0xFFFFU;
  uint16_t vmax = 0U;
  uint8_t i;
  bool ok = false;

  if (out == NULL)
  {
    return false;
  }

  out->valid = false;

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  for (i = 0U; i < BQ76942_CELL_COUNT; i++)
  {
    if (!BQ76942_ReadDirectU16(hi2c, (uint8_t)(BQ76942_CMD_CELL1_VOLTAGE + (2U * i)), &raw_u16))
    {
      goto out;
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
    goto out;
  }
  out->pack_mv = BQ76942_UserVToMv(raw_u16);

  if (!BQ76942_ReadDirectU16(hi2c, BQ76942_CMD_PACK_VOLTAGE, &raw_u16))
  {
    goto out;
  }
  out->output_mv = BQ76942_UserVToMv(raw_u16);

  if (!BQ76942_ReadDirectU16(hi2c, BQ76942_CMD_LD_VOLTAGE, &raw_u16))
  {
    goto out;
  }
  out->ld_mv = BQ76942_UserVToMv(raw_u16);

  if (!BQ76942_ReadDirectS16(hi2c, BQ76942_CMD_CC2_CURRENT, &raw_s16))
  {
    goto out;
  }
  out->current_ma = raw_s16;

  /* CC3: averaged CC2 samples via DASTATUS5 bytes 20–21. */
  {
    uint8_t dastatus5[BQ76942_DASTATUS5_CC3_OFFSET + 2U];

    if (!BQ76942_SubCommandRead(hi2c, BQ76942_SUBCMD_DASTATUS5,
                                dastatus5, (uint8_t)sizeof(dastatus5)))
    {
      goto out;
    }

    out->current_cc3_ma = (int16_t)((uint16_t)dastatus5[BQ76942_DASTATUS5_CC3_OFFSET] |
                                    ((uint16_t)dastatus5[BQ76942_DASTATUS5_CC3_OFFSET + 1U] << 8));
  }

  out->vcell_min_mv = vmin;
  out->vcell_max_mv = vmax;
  out->valid = true;
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_ReadStackOutputMv(I2C_HandleTypeDef *hi2c, uint32_t *stack_mv,
                               uint32_t *output_mv)
{
  uint16_t raw_u16;
  bool ok = false;

  if ((hi2c == NULL) || (stack_mv == NULL) || (output_mv == NULL))
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_ReadDirectU16(hi2c, BQ76942_CMD_STACK_VOLTAGE, &raw_u16))
  {
    goto out;
  }
  *stack_mv = BQ76942_UserVToMv(raw_u16);

  if (!BQ76942_ReadDirectU16(hi2c, BQ76942_CMD_PACK_VOLTAGE, &raw_u16))
  {
    goto out;
  }
  *output_mv = BQ76942_UserVToMv(raw_u16);
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_ReadFetStatus(I2C_HandleTypeDef *hi2c, uint8_t *fet_status)
{
  if ((hi2c == NULL) || (fet_status == NULL))
  {
    return false;
  }

  return (BqI2cMemRead(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_CMD_FET_STATUS,
                           I2C_MEMADD_SIZE_8BIT, fet_status, 1U,
                           BQ76942_I2C_TIMEOUT_MS) == HAL_OK);
}

static bool BQ76942_WriteFetPredischargeConfig(I2C_HandleTypeDef *hi2c)
{
  uint8_t fet_options = 0x0DU;
  uint8_t timeout = BQ76942_PREDISCHARGE_TIMEOUT;
  uint8_t stop_delta = BQ76942_PREDISCHARGE_STOP_DELTA;
  uint8_t pin_cfg = BQ76942_PINCFG_FETOFF_ACTIVE_HIGH;
  bool ok = false;

  fet_options = (uint8_t)(fet_options | BQ76942_FETOPT_PDSG_EN);

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_CFETOFF_PIN_CONFIG,
                               &pin_cfg, 1U))
  {
    goto out;
  }

  if (!BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_DFETOFF_PIN_CONFIG,
                               &pin_cfg, 1U))
  {
    goto out;
  }

  if (!BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_FET_OPTIONS, &fet_options, 1U))
  {
    goto out;
  }

  if (!BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_PREDISCHARGE_TIMEOUT,
                               &timeout, 1U))
  {
    goto out;
  }

  ok = BQ76942_DataMemoryWrite(hi2c, BQ76942_DM_PREDISCHARGE_STOP_DELTA,
                               &stop_delta, 1U);

out:
  BQ76942_I2cUnlock();
  return ok;
}

static bool BQ76942_SubCommandWriteU16(I2C_HandleTypeDef *hi2c, uint16_t subcmd,
                                       uint16_t value)
{
  uint8_t block[4];
  uint8_t meta[2];
  bool ok = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  block[0] = (uint8_t)(subcmd & 0xFFU);
  block[1] = (uint8_t)((subcmd >> 8) & 0xFFU);
  block[2] = (uint8_t)(value & 0xFFU);
  block[3] = (uint8_t)((value >> 8) & 0xFFU);

  if (BqI2cMemWrite(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_CMD_LOW,
                        I2C_MEMADD_SIZE_8BIT, block, sizeof(block),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  meta[0] = Bq76942_DmChecksum(block, (uint8_t)sizeof(block));
  meta[1] = (uint8_t)(sizeof(block) + 2U);
  if (BqI2cMemWrite(hi2c, BQ76942_I2C_ADDR_HAL, 0x60U,
                        I2C_MEMADD_SIZE_8BIT, meta, sizeof(meta),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  osDelay(BQ76942_SUBCMD_WAIT_MS);
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

static bool BQ76942_SubCommandWriteU8(I2C_HandleTypeDef *hi2c, uint16_t subcmd,
                                      uint8_t data)
{
  uint8_t block[3];
  uint8_t meta[2];
  bool ok = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  block[0] = (uint8_t)(subcmd & 0xFFU);
  block[1] = (uint8_t)((subcmd >> 8) & 0xFFU);
  block[2] = data;

  if (BqI2cMemWrite(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_REG_CMD_LOW,
                        I2C_MEMADD_SIZE_8BIT, block, sizeof(block),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  meta[0] = Bq76942_DmChecksum(block, (uint8_t)sizeof(block));
  meta[1] = (uint8_t)(sizeof(block) + 2U); /* cmd(2)+data(1)+0x60/0x61 */
  if (BqI2cMemWrite(hi2c, BQ76942_I2C_ADDR_HAL, 0x60U,
                        I2C_MEMADD_SIZE_8BIT, meta, sizeof(meta),
                        BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  osDelay(BQ76942_SUBCMD_WAIT_MS);
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

static bool BQ76942_EnsureFetDriverReady(I2C_HandleTypeDef *hi2c)
{
  uint16_t mfg_status = 0U;
  bool ok = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_SubCommandReadU16(hi2c, BQ76942_SUBCMD_MFG_STATUS, &mfg_status))
  {
    goto out;
  }

  if ((mfg_status & BQ76942_MFG_FET_EN) == 0U)
  {
    if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_FET_ENABLE))
    {
      goto out;
    }
    osDelay(10);
  }

  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

static bool BQ76942_EnsureFetsEnabled(I2C_HandleTypeDef *hi2c)
{
  bool ok = false;

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_EnsureFetDriverReady(hi2c))
  {
    goto out;
  }

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_ALL_FETS_ON))
  {
    goto out;
  }

  osDelay(10);
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_EnablePreDischargePath(I2C_HandleTypeDef *hi2c)
{
  uint8_t fet_status = 0U;
  uint8_t retry;
  bool pdsg_seen = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  /* DFETOFF must be released by charge_path before calling. */
  if (!BQ76942_EnsureFetDriverReady(hi2c))
  {
    goto out;
  }

  /* 复位 FET 状态，避免仅 CHG 侧开启(如 0x73)时 PDSG 不再拉起。 */
  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_ALL_FETS_OFF))
  {
    goto out;
  }
  osDelay(10);

  /* 先仅允许 PDSG，等 PDSG 置位后再放开 DSG，避免 LD 已贴近时 DSG 被挡住。 */
  if (!BQ76942_SubCommandWriteU8(hi2c, BQ76942_SUBCMD_FET_CONTROL,
                                 BQ76942_FETCTRL_PDSG_ONLY))
  {
    goto out;
  }

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_ALL_FETS_ON))
  {
    goto out;
  }

  for (retry = 0U; retry < BQ76942_FET_ON_RETRY; retry++)
  {
    osDelay(BQ76942_FET_ON_WAIT_MS);

    if (!BQ76942_ReadFetStatus(hi2c, &fet_status))
    {
      continue;
    }

    if ((fet_status & (BQ76942_FETSTAT_PDSG_FET | BQ76942_FETSTAT_DSG_FET)) != 0U)
    {
      pdsg_seen = true;
      break;
    }
  }

  if (!BQ76942_SubCommandWriteU8(hi2c, BQ76942_SUBCMD_FET_CONTROL,
                                 BQ76942_FETCTRL_ALLOW_ALL))
  {
    goto out;
  }

  if (!BQ76942_SubCommandWrite(hi2c, BQ76942_SUBCMD_ALL_FETS_ON))
  {
    goto out;
  }

  for (retry = 0U; retry < BQ76942_FET_ON_RETRY; retry++)
  {
    osDelay(BQ76942_FET_ON_WAIT_MS);

    if (!BQ76942_ReadFetStatus(hi2c, &fet_status))
    {
      continue;
    }

    if ((fet_status & (BQ76942_FETSTAT_PDSG_FET | BQ76942_FETSTAT_DSG_FET)) != 0U)
    {
      pdsg_seen = true;
      break;
    }
  }

out:
  BQ76942_I2cUnlock();
  return pdsg_seen;
}

bool BQ76942_EnableDischargePath(I2C_HandleTypeDef *hi2c)
{
  bool ok = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  /* 24V bypass unused. DFETOFF owned by charge_path. */
  if (!BQ76942_EnsureFetsEnabled(hi2c))
  {
    goto out;
  }

  /* ALL_FETS_ON 已发出；DSG 可能尚未从 PDSG 切过来。 */
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_EnableChargePath(I2C_HandleTypeDef *hi2c)
{
  uint8_t fet_status = 0U;
  bool ok = false;

  if (hi2c == NULL)
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (!BQ76942_EnsureFetsEnabled(hi2c))
  {
    goto out;
  }

  if (!BQ76942_ReadFetStatus(hi2c, &fet_status))
  {
    goto out;
  }

  ok = ((fet_status & BQ76942_FETSTAT_CHG_FET) != 0U);

out:
  BQ76942_I2cUnlock();
  return ok;
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
bool BQ76942_ReadSafetyStatusEx(I2C_HandleTypeDef *hi2c,
                                uint8_t *status_a, uint8_t *status_b,
                                uint8_t *status_c)
{
  bool ok = false;

  if ((hi2c == NULL) || (status_a == NULL) || (status_b == NULL) ||
      (status_c == NULL))
  {
    return false;
  }

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  if (BqI2cMemRead(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_CMD_SAFETY_STATUS_A,
                       I2C_MEMADD_SIZE_8BIT, status_a, 1U,
                       BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  if (BqI2cMemRead(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_CMD_SAFETY_STATUS_B,
                       I2C_MEMADD_SIZE_8BIT, status_b, 1U,
                       BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  if (BqI2cMemRead(hi2c, BQ76942_I2C_ADDR_HAL, BQ76942_CMD_SAFETY_STATUS_C,
                       I2C_MEMADD_SIZE_8BIT, status_c, 1U,
                       BQ76942_I2C_TIMEOUT_MS) != HAL_OK)
  {
    goto out;
  }

  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

bool BQ76942_ReadSafetyStatus(I2C_HandleTypeDef *hi2c, bool *protect_active)
{
  uint8_t sa = 0U;
  uint8_t sb = 0U;
  uint8_t sc = 0U;

  if (protect_active == NULL)
  {
    return false;
  }

  *protect_active = false;
  if (!BQ76942_ReadSafetyStatusEx(hi2c, &sa, &sb, &sc))
  {
    return false;
  }

  *protect_active = ((sa != 0U) || (sb != 0U) || (sc != 0U));
  return true;
}

bool BQ76942_SetBalanceMask(I2C_HandleTypeDef *hi2c, uint16_t mask)
{
  bool ok;

  if (hi2c == NULL)
  {
    return false;
  }

  ok = BQ76942_SubCommandWriteU16(hi2c, BQ76942_SUBCMD_CB_ACTIVE_CELLS, mask);
  if (ok)
  {
    osDelay(BQ76942_BALANCE_SUBCMD_WAIT_MS);
  }

  return ok;
}

bool BQ76942_ReadBalanceMask(I2C_HandleTypeDef *hi2c, uint16_t *mask)
{
  if ((hi2c == NULL) || (mask == NULL))
  {
    return false;
  }

  return BQ76942_SubCommandReadU16(hi2c, BQ76942_SUBCMD_CB_ACTIVE_CELLS, mask);
}

bool BQ76942_ReadBalanceActiveSec(I2C_HandleTypeDef *hi2c, uint16_t *seconds)
{
  if ((hi2c == NULL) || (seconds == NULL))
  {
    return false;
  }

  return BQ76942_SubCommandReadU16(hi2c, BQ76942_SUBCMD_CBSTATUS1, seconds);
}
