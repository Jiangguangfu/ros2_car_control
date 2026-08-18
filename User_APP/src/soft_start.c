/**
 ******************************************************************************
 * @file    soft_start.c
 * @brief   BMS 上电缓启动与自检
 ******************************************************************************
 */
#include "soft_start.h"

#include "bsp_power_rails.h"
#include "bsp_adc_rails.h"
#include "charge_path.h"
#include "cmsis_os2.h"
#include "SEGGER_RTT.h"

#include <stdio.h>
#include <string.h>

#define POSC_PACK_STACK_DELTA_MV          500U
#define POSC_DSG_SETTLE_MS                250U
#define POSC_RAIL_PGOOD_TIMEOUT_MS        2000U
#define POSC_S0_I2C_RETRY_MS              100U
#define POSC_S1_PRECHARGE_TIMEOUT_MS      8000U
#define POSC_S1_POLL_MS                   50U
#define POSC_S2_DSG_TIMEOUT_MS            2000U
#define POSC_I2C_RETRY_COUNT              3U
#define POSC_I2C_RETRY_DELAY_MS           20U
#define POSC_STATE_CONFIRM_COUNT          3U
#define POSC_DFETOFF_SETTLE_MS            50U
#define POSC_PDSG_RECOMMAND_MS            500U
#define POSC_OUTPUT_MIN_RISE_MV           200U

extern I2C_HandleTypeDef hi2c2;

static volatile bool s_system_ready;
static volatile bool s_boot_fault;
static bool s_bq_calibrated;
static posc_snapshot_t s_posc;
static int16_t s_current_prev_ma;

static void SoftStart_LogPuts(const char *s)
{
  (void)SEGGER_RTT_WriteString(0U, s);
}

static bool SoftStart_ReadPackCurrent(int16_t *current_ma)
{
  uint8_t retry;

  for (retry = 0U; retry < POSC_I2C_RETRY_COUNT; retry++)
  {
    if (BQ76942_ReadPackCurrent(&hi2c2, current_ma))
    {
      s_posc.meas.current_ma = *current_ma;
      s_posc.current_valid = true;
      return true;
    }

    if ((retry + 1U) < POSC_I2C_RETRY_COUNT)
    {
      osDelay(POSC_I2C_RETRY_DELAY_MS);
    }
  }

  return false;
}

/** 打印 PACK 总电流、各轨 ADC 电压与 EN 状态；dI 为相对上一次采样的差值。 */
static void SoftStart_LogRailCurrents(const char *phase)
{
  int16_t pack_ma = 0;
  int16_t delta_ma = 0;
  char line[280];
  const pwr_rails_status_t *pwr = BSP_PowerRails_GetStatus();
  const adc_rails_status_t *adc = BSP_AdcRails_GetStatus();
  bool i_ok = SoftStart_ReadPackCurrent(&pack_ma);
  uint32_t v12_mv = 0U;
  uint32_t v19_mv = 0U;
  uint32_t v75_mv = 0U;
  uint32_t v24_mv = 0U;
  uint32_t i5_ma = 0U;
  uint16_t raw[BSP_ADC_CHANNEL_COUNT] = {0};
  uint8_t on_12v = 0U;
  uint8_t on_19v = 0U;
  uint8_t on_6v5 = 0U;
  uint8_t on_24v = 0U;
  uint8_t i;

  BSP_AdcRails_Update();
  adc = BSP_AdcRails_GetStatus();

  if ((adc != NULL) && adc->ready)
  {
    v12_mv = adc->rail_mv[PWR_RAIL_12V];
    v19_mv = adc->rail_mv[PWR_RAIL_19V];
    v75_mv = adc->rail_mv[PWR_RAIL_6V5];
    v24_mv = adc->rail_mv[PWR_RAIL_24V];
    i5_ma = adc->i5v_ma;
    for (i = 0U; i < BSP_ADC_CHANNEL_COUNT; i++)
    {
      raw[i] = adc->channel_raw[i];
    }
  }

  if (pwr != NULL)
  {
    on_12v = pwr->rail_on[PWR_RAIL_12V] ? 1U : 0U;
    on_19v = pwr->rail_on[PWR_RAIL_19V] ? 1U : 0U;
    on_6v5 = pwr->rail_on[PWR_RAIL_6V5] ? 1U : 0U;
    on_24v = pwr->rail_on[PWR_RAIL_24V] ? 1U : 0U;
  }

  if (i_ok)
  {
    delta_ma = (int16_t)(pack_ma - s_current_prev_ma);
    s_current_prev_ma = pack_ma;
    (void)snprintf(line, sizeof(line),
                   "[POSC] %s Ipack=%d mA dI=%d mA "
                   "V12=%lu V19=%lu V75=%lu V24=%lu mV I5=%lu mA "
                   "EN12=%u EN19=%u EN6V5=%u EN24=%u "
                   "raw=%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                   phase, (int)pack_ma, (int)delta_ma,
                   (unsigned long)v12_mv, (unsigned long)v19_mv,
                   (unsigned long)v75_mv, (unsigned long)v24_mv,
                   (unsigned long)i5_ma,
                   (unsigned)on_12v, (unsigned)on_19v,
                   (unsigned)on_6v5, (unsigned)on_24v,
                   (unsigned)raw[0], (unsigned)raw[1], (unsigned)raw[2],
                   (unsigned)raw[3], (unsigned)raw[4], (unsigned)raw[5],
                   (unsigned)raw[6], (unsigned)raw[7], (unsigned)raw[8]);
  }
  else
  {
    (void)snprintf(line, sizeof(line),
                   "[POSC] %s Ipack=NA "
                   "V12=%lu V19=%lu V75=%lu V24=%lu mV I5=%lu mA "
                   "EN12=%u EN19=%u EN6V5=%u EN24=%u "
                   "raw=%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                   phase,
                   (unsigned long)v12_mv, (unsigned long)v19_mv,
                   (unsigned long)v75_mv, (unsigned long)v24_mv,
                   (unsigned long)i5_ma,
                   (unsigned)on_12v, (unsigned)on_19v,
                   (unsigned)on_6v5, (unsigned)on_24v,
                   (unsigned)raw[0], (unsigned)raw[1], (unsigned)raw[2],
                   (unsigned)raw[3], (unsigned)raw[4], (unsigned)raw[5],
                   (unsigned)raw[6], (unsigned)raw[7], (unsigned)raw[8]);
  }

  SoftStart_LogPuts(line);
}

static bool SoftStart_TempValid(int16_t t_c_x10)
{
  return (t_c_x10 > -400) && (t_c_x10 < 1250);
}

bool SoftStart_IsSystemReady(void)
{
  return s_system_ready;
}

bool SoftStart_IsBootFault(void)
{
  return s_boot_fault;
}

bool SoftStart_IsBqCalibrated(void)
{
  return s_bq_calibrated;
}

void SoftStart_SetBqCalibrated(bool calibrated)
{
  s_bq_calibrated = calibrated;
}

const posc_snapshot_t *SoftStart_GetSnapshot(void)
{
  return &s_posc;
}

uint8_t SoftStart_GetFetStatus(void)
{
  return s_posc.fet_status;
}

void SoftStart_Init(void)
{
  (void)memset(&s_posc, 0, sizeof(s_posc));
  s_posc.state = POSC_S0_SELF_CHECK;
  s_system_ready = false;
  s_boot_fault = false;
  s_bq_calibrated = false;
  s_posc.rail_24v_ok = false;
  s_posc.current_valid = false;
  s_current_prev_ma = 0;
  (void)BSP_PowerRails_EnableRail(PWR_RAIL_24V, false);
  ChargePath_Init();
  SoftStart_LogPuts("[POSC] boot start\r\n");
}

/** 更新 FET 快照；读失败时保留上次 fet_status，不清零。 */
static void SoftStart_UpdateFetStatus(void)
{
  uint8_t fet_status = 0U;
  uint8_t retry;

  for (retry = 0U; retry < POSC_I2C_RETRY_COUNT; retry++)
  {
    if (BQ76942_ReadFetStatus(&hi2c2, &fet_status))
    {
      s_posc.fet_valid = true;
      s_posc.fet_status = fet_status;
      return;
    }

    if ((retry + 1U) < POSC_I2C_RETRY_COUNT)
    {
      osDelay(POSC_I2C_RETRY_DELAY_MS);
    }
  }

  s_posc.fet_valid = false;
}

/** S1 轻量采样：FET + Stack/PACK 输出。 */
static void SoftStart_ReadChipS1(void)
{
  uint32_t stack_mv = 0U;
  uint32_t output_mv = 0U;
  uint8_t retry;

  for (retry = 0U; retry < POSC_I2C_RETRY_COUNT; retry++)
  {
    if (BQ76942_I2cLock())
    {
      s_posc.i2c_ready = BQ76942_IsReady(&hi2c2);
      if (s_posc.i2c_ready)
      {
        SoftStart_UpdateFetStatus();

        if (BQ76942_ReadStackOutputMv(&hi2c2, &stack_mv, &output_mv))
        {
          s_posc.meas.pack_mv = stack_mv;
          s_posc.meas.output_mv = output_mv;
          s_posc.meas.valid = true;
          BQ76942_I2cUnlock();
          return;
        }
      }

      BQ76942_I2cUnlock();
    }

    if ((retry + 1U) < POSC_I2C_RETRY_COUNT)
    {
      osDelay(POSC_I2C_RETRY_DELAY_MS);
    }
  }

  s_posc.i2c_ready = false;
  s_posc.meas.valid = false;
}

/** 直接从 BQ 读 I2C/保护/温度/测量/FET，不依赖其他模块缓存。 */
static void SoftStart_ReadChip(void)
{
  if (!BQ76942_I2cLock())
  {
    s_posc.i2c_ready = false;
    s_posc.safety_valid = false;
    s_posc.temp.valid = false;
    s_posc.meas.valid = false;
    s_posc.fet_valid = false;
    return;
  }

  s_posc.i2c_ready = BQ76942_IsReady(&hi2c2);
  if (!s_posc.i2c_ready)
  {
    s_posc.safety_valid = false;
    s_posc.temp.valid = false;
    s_posc.meas.valid = false;
    s_posc.fet_valid = false;
    BQ76942_I2cUnlock();
    return;
  }

  s_posc.safety_valid = BQ76942_ReadSafetyStatusEx(&hi2c2, &s_posc.safety_a,
                                                   &s_posc.safety_b,
                                                   &s_posc.safety_c);
  if (!s_posc.safety_valid)
  {
    s_posc.safety_a = 0U;
    s_posc.safety_b = 0U;
    s_posc.safety_c = 0U;
  }

  if (!BQ76942_ReadTemperatures(&hi2c2, &s_posc.temp))
  {
    s_posc.temp.valid = false;
  }

  SoftStart_UpdateFetStatus();

  if (!BQ76942_ReadMeasurements(&hi2c2, &s_posc.meas))
  {
    s_posc.meas.valid = false;
  }

  BQ76942_I2cUnlock();
}

/** 步骤 0：检查 I2C、故障位、温度及测量值。 */
static bool SoftStart_S0(void)
{
  bool ts1_ok;
  bool ts2_ok;
  bool ok = false;

  if (!BQ76942_I2cLock())
  {
    return false;
  }

  SoftStart_ReadChip();
  if (!s_posc.i2c_ready)
  {
    goto out;
  }

  if (!BQ76942_InitCalibration(&hi2c2))
  {
    s_posc.calibrated = false;
    goto out;
  }
  s_posc.calibrated = true;

  SoftStart_ReadChip();
  if (!s_posc.safety_valid)
  {
    goto out;
  }

  if ((s_posc.safety_a & (BQ76942_SA_COV | BQ76942_SA_CUV)) != 0U)
  {
    goto out;
  }

  if ((s_posc.safety_a &
       (BQ76942_SA_SCD | BQ76942_SA_OCD1 | BQ76942_SA_OCD2)) != 0U)
  {
    goto out;
  }

  if (!s_posc.temp.valid)
  {
    goto out;
  }

  ts1_ok = SoftStart_TempValid(s_posc.temp.ts1_temp_c_x10);
  ts2_ok = SoftStart_TempValid(s_posc.temp.ts2_temp_c_x10);
  if (!ts1_ok && !ts2_ok &&
      !SoftStart_TempValid(s_posc.temp.int_temp_c_x10))
  {
    goto out;
  }

  if (!s_posc.meas.valid)
  {
    goto out;
  }

  s_bq_calibrated = true;
  ok = true;

out:
  BQ76942_I2cUnlock();
  return ok;
}

/** 用 RTOS tick 计算真实经过时间，不受 osDelay 被拉长影响。 */
static uint32_t SoftStart_ElapsedMs(uint32_t start_tick)
{
  uint32_t ticks = osKernelGetTickCount() - start_tick;
  uint32_t freq = osKernelGetTickFreq();

  if (freq == 0U)
  {
    return ticks;
  }

  return (uint32_t)(((uint64_t)ticks * 1000U) / freq);
}

/** PACK 输出贴近 Stack：输出电容已充满。 */
static bool SoftStart_OutputNearStack(void)
{
  return s_posc.meas.valid &&
         (s_posc.meas.pack_mv > POSC_PACK_STACK_DELTA_MV) &&
         ((s_posc.meas.output_mv + POSC_PACK_STACK_DELTA_MV) >=
          s_posc.meas.pack_mv);
}

/** 0x73：仅 CHG/PCHG 开，DSG/PDSG 关 — 预放电未启动。 */
static bool SoftStart_FetChargeOnlyNoDischarge(void)
{
  return s_posc.fet_valid &&
         ((s_posc.fet_status & BQ76942_FETSTAT_CHG_SIDE) != 0U) &&
         ((s_posc.fet_status & BQ76942_FETSTAT_DSG_SIDE) == 0U);
}

static bool SoftStart_FetDischargeActive(void)
{
  return s_posc.fet_valid &&
         ((s_posc.fet_status & BQ76942_FETSTAT_DSG_SIDE) != 0U);
}

static bool SoftStart_FetDsgActive(void)
{
  return s_posc.fet_valid &&
         ((s_posc.fet_status & BQ76942_FETSTAT_DSG_FET) != 0U);
}

static bool SoftStart_OutputRisenFrom(uint32_t baseline_mv)
{
  return s_posc.meas.valid &&
         (s_posc.meas.output_mv >= (baseline_mv + POSC_OUTPUT_MIN_RISE_MV));
}

/** S1 完成：DSG 已开，或 PACK 贴近 Stack 且放电侧有响应 / 输出抬升。 */
static bool SoftStart_S1StepDone(bool saw_dsg_side, uint32_t baseline_mv)
{
  if (SoftStart_FetDsgActive())
  {
    return true;
  }

  if (!SoftStart_OutputNearStack())
  {
    return false;
  }

  return saw_dsg_side || SoftStart_OutputRisenFrom(baseline_mv);
}

/** S2 完成：DSG 已开，或曾见 PDSG/DSG 且 output 贴近 Stack。 */
static bool SoftStart_S2StepDone(bool saw_dsg_side)
{
  if (SoftStart_FetDsgActive())
  {
    return true;
  }

  return saw_dsg_side && SoftStart_OutputNearStack();
}

static bool SoftStart_EnablePreDischargeWithRetry(void)
{
  uint8_t retry;

  for (retry = 0U; retry < POSC_I2C_RETRY_COUNT; retry++)
  {
    if (BQ76942_EnablePreDischargePath(&hi2c2))
    {
      return true;
    }

    if ((retry + 1U) < POSC_I2C_RETRY_COUNT)
    {
      osDelay(POSC_I2C_RETRY_DELAY_MS);
    }
  }

  return false;
}

static bool SoftStart_EnableDischargeWithRetry(void)
{
  uint8_t retry;

  for (retry = 0U; retry < POSC_I2C_RETRY_COUNT; retry++)
  {
    if (BQ76942_EnableDischargePath(&hi2c2))
    {
      return true;
    }

    if ((retry + 1U) < POSC_I2C_RETRY_COUNT)
    {
      osDelay(POSC_I2C_RETRY_DELAY_MS);
    }
  }

  return false;
}

/** 步骤 1：PDSG 预充。不开 24V bypass，只等放电侧 FET / PACK 输出抬升。 */
static bool SoftStart_S1(void)
{
  uint32_t start_tick;
  uint32_t last_pdsg_cmd_tick;
  uint32_t baseline_output_mv = 0U;
  bool saw_dsg_side = false;
  uint8_t confirm_count = 0U;

  s_posc.s1_elapsed_ms = 0U;
  s_posc.rail_24v_ok = false;
  (void)BSP_PowerRails_EnableRail(PWR_RAIL_24V, false);

  ChargePath_SetBootDischargeInhibit(false);
  ChargePath_Apply();
  osDelay(POSC_DFETOFF_SETTLE_MS);

  if (ChargePath_IsDischargeInhibited())
  {
    return false;
  }

  SoftStart_ReadChipS1();
  if (s_posc.meas.valid)
  {
    baseline_output_mv = s_posc.meas.output_mv;
  }

  if (!SoftStart_EnablePreDischargeWithRetry())
  {
    return false;
  }

  start_tick = osKernelGetTickCount();
  last_pdsg_cmd_tick = start_tick;
  while (s_posc.s1_elapsed_ms < POSC_S1_PRECHARGE_TIMEOUT_MS)
  {
    SoftStart_ReadChipS1();

    if (SoftStart_FetDischargeActive())
    {
      saw_dsg_side = true;
    }

    if (SoftStart_FetChargeOnlyNoDischarge() &&
        (SoftStart_ElapsedMs(last_pdsg_cmd_tick) >= POSC_PDSG_RECOMMAND_MS))
    {
      ChargePath_SetBootDischargeInhibit(false);
      ChargePath_Apply();
      (void)SoftStart_EnablePreDischargeWithRetry();
      last_pdsg_cmd_tick = osKernelGetTickCount();
    }

    if (SoftStart_S1StepDone(saw_dsg_side, baseline_output_mv))
    {
      confirm_count++;
      if (confirm_count >= POSC_STATE_CONFIRM_COUNT)
      {
        s_posc.s1_elapsed_ms = SoftStart_ElapsedMs(start_tick);
        return true;
      }
    }
    else
    {
      confirm_count = 0U;
    }

    osDelay(POSC_S1_POLL_MS);
    s_posc.s1_elapsed_ms = SoftStart_ElapsedMs(start_tick);
  }

  SoftStart_ReadChipS1();
  s_posc.s1_elapsed_ms = SoftStart_ElapsedMs(start_tick);
  return false;
}

/** 步骤 2：等待芯片从 PDSG 切换至 DSG。 */
static bool SoftStart_S2(void)
{
  uint32_t start_tick = osKernelGetTickCount();
  uint32_t last_dsg_cmd_tick = start_tick;
  bool saw_dsg_side = false;
  uint8_t confirm_count = 0U;

  ChargePath_Apply();
  if (!SoftStart_EnableDischargeWithRetry())
  {
    return false;
  }

  while (SoftStart_ElapsedMs(start_tick) < POSC_S2_DSG_TIMEOUT_MS)
  {
    SoftStart_ReadChipS1();

    if (SoftStart_FetDischargeActive())
    {
      saw_dsg_side = true;
    }

    if (SoftStart_S2StepDone(saw_dsg_side))
    {
      confirm_count++;
      if (confirm_count >= POSC_STATE_CONFIRM_COUNT)
      {
        osDelay(POSC_DSG_SETTLE_MS);
        return true;
      }
    }
    else if (SoftStart_FetChargeOnlyNoDischarge() &&
             (SoftStart_ElapsedMs(last_dsg_cmd_tick) >= POSC_PDSG_RECOMMAND_MS))
    {
      ChargePath_SetBootDischargeInhibit(false);
      ChargePath_Apply();
      (void)SoftStart_EnableDischargeWithRetry();
      last_dsg_cmd_tick = osKernelGetTickCount();
      confirm_count = 0U;
    }
    else
    {
      confirm_count = 0U;
    }

    osDelay(POSC_S1_POLL_MS);
  }

  SoftStart_ReadChipS1();
  return false;
}

static void SoftStart_EnterFault(void)
{
  s_posc.state = POSC_FAULT;
  s_boot_fault = true;
  s_system_ready = false;
  /* 先打日志再关轨，否则 19V 电压已经掉下去了。 */
  SoftStart_LogRailCurrents("fault");
  ChargePath_SetBootDischargeInhibit(true);
  ChargePath_Apply();
  s_posc.rail_24v_ok = false;
  (void)BSP_PowerRails_EnableRail(PWR_RAIL_24V, false);
  (void)BSP_PowerRails_EnableRail(PWR_RAIL_12V, false);
  (void)BSP_PowerRails_EnableRail(PWR_RAIL_19V, false);
  (void)BSP_PowerRails_EnableRail(PWR_RAIL_6V5, false);
  BSP_PowerRails_SetBootComplete(false);
}

static void SoftStart_EnterReady(void)
{
  s_posc.state = POSC_READY;
  s_system_ready = true;
  s_boot_fault = false;
  s_posc.rail_24v_ok = false;
  (void)BSP_PowerRails_EnableRail(PWR_RAIL_24V, false);
  ChargePath_SetBootDischargeInhibit(false);
  ChargePath_Apply();
  BSP_PowerRails_SetBootComplete(true);
  SoftStart_LogRailCurrents("after");
  if (s_posc.current_valid)
  {
    s_posc.current_after_19v_ma = s_posc.meas.current_ma;
  }
}

void SoftStart_Process(void)
{
  switch (s_posc.state)
  {
    case POSC_S0_SELF_CHECK:
      if (SoftStart_S0())
      {
        SoftStart_LogRailCurrents("before");
        if (s_posc.current_valid)
        {
          s_posc.current_before_ma = s_posc.meas.current_ma;
        }
        s_posc.state = POSC_S1_PDSG;
      }
      else
      {
        osDelay(POSC_S0_I2C_RETRY_MS);
      }
      break;

    case POSC_S1_PDSG:
      if (SoftStart_S1())
      {
        s_posc.state = POSC_S2_DSG;
      }
      else
      {
        SoftStart_EnterFault();
      }
      break;

    case POSC_S2_DSG:
      if (SoftStart_S2())
      {
        SoftStart_LogRailCurrents("after_dsg");
        s_posc.state = POSC_S3_12V;
      }
      else
      {
        SoftStart_EnterFault();
      }
      break;

    case POSC_S3_12V:
      /* 与 6.5V/19V 相同：EN 拉起失败才 FAULT。PA0 12V 分压在本板常读
       * ~0 mV（raw=2），ADC 超时不能把整机卡死。 */
      if (!BSP_PowerRails_EnableRail(PWR_RAIL_12V, true))
      {
        SoftStart_EnterFault();
        break;
      }
      s_posc.rail_12v_ok =
          BSP_PowerRails_WaitRailGood(PWR_RAIL_12V,
                                      POSC_RAIL_PGOOD_TIMEOUT_MS);
      SoftStart_LogRailCurrents(s_posc.rail_12v_ok ? "after_12V" :
                                "after_12V_adc");
      if (s_posc.current_valid)
      {
        s_posc.current_after_12v_ma = s_posc.meas.current_ma;
      }
      s_posc.state = POSC_S4_OTHER;
      break;

    case POSC_S4_OTHER:
      /* 6.5V 灯/输出已确认能起来；PA6 分压通道尚未按 DMA 对齐后的 raw 标定，
       * ADC 超时只记日志，不进 FAULT，避免卡在 S4。 */
      if (!BSP_PowerRails_EnableRail(PWR_RAIL_6V5, true))
      {
        SoftStart_EnterFault();
        break;
      }
      s_posc.rail_6v5_ok =
          BSP_PowerRails_WaitRailGood(PWR_RAIL_6V5,
                                      POSC_RAIL_PGOOD_TIMEOUT_MS);
      SoftStart_LogRailCurrents(s_posc.rail_6v5_ok ? "after_6V5" :
                                "after_6V5_adc");
      if (s_posc.current_valid)
      {
        s_posc.current_after_6v5_ma = s_posc.meas.current_ma;
      }
      s_posc.state = POSC_S5_19V;
      break;

    case POSC_S5_19V:
      /* 19V 灯已能亮；ADC 分压/通道尚未按对齐后的 raw 标定，超时不关轨。 */
      if (!BSP_PowerRails_EnableRail(PWR_RAIL_19V, true))
      {
        SoftStart_EnterFault();
        break;
      }
      s_posc.rail_19v_ok =
          BSP_PowerRails_WaitRailGood(PWR_RAIL_19V,
                                      POSC_RAIL_PGOOD_TIMEOUT_MS);
      SoftStart_LogRailCurrents(s_posc.rail_19v_ok ? "after_19V" :
                                "after_19V_adc");
      SoftStart_EnterReady();
      break;

    case POSC_READY:
    case POSC_FAULT:
    default:
      break;
  }
}
