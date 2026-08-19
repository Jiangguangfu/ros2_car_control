/**
 ******************************************************************************
 * @file    bsp_adc_rails.c
 * @brief   电源轨 ADC 电压 / 电流监测（分压比与增益按原理图）
 ******************************************************************************
 */
#include "bsp_adc_rails.h"

#include "main.h"

#include <string.h>

#define BSP_ADC_VREF_MV           3300U
#define BSP_ADC_MAX_RAW           4095U

/* DMA Rank 索引（0-based，与 MX_ADC1_Init Rank 顺序一致） */
#define BSP_ADC_IDX_24V_I         0U  /* PA0 IN3  ADC_M_24V_O1 电流 */
#define BSP_ADC_IDX_19V_I         1U  /* PA1 IN4  ADC_M_19V_O1 电流 */
#define BSP_ADC_IDX_12V_I         2U  /* PA2 IN5  ADC_M_12V_O1 电流（遥测） */
#define BSP_ADC_IDX_5V_I          3U  /* PA3 IN6  ADC_M_5V 电流 */
#define BSP_ADC_IDX_7V5_I         4U  /* PA6 IN9  ADC_M_7.5V_O1 电流（遥测） */
#define BSP_ADC_IDX_7V5_V         5U  /* PA7 IN10 +7.5V_OUT3_V 电压 */
#define BSP_ADC_IDX_12V_V         6U  /* PB0 IN13 +12V_OUT2_V 电压 */

/*
 * 电压：Vrail_mV = raw * VREF * (Rtop + Rbot) / (ADC_MAX * Rbot)
 *
 *   12V  PB0 IN13 R104=120k / R102=20k → 7/1
 *   6.5V PA7 IN10 R49=40k / R105=10k   → 5/1
 *   19V/24V 无分压通道，不换算电压。
 */
#define BSP_ADC_DIV_12_NUM         7U
#define BSP_ADC_DIV_12_DEN         1U
#define BSP_ADC_DIV_6V5_NUM        5U
#define BSP_ADC_DIV_6V5_DEN        1U

/*
 * 电流：INA180A2（Gain=50）× Rshunt=10 mΩ
 * I_mA = Vadc_mV * 1000 / (Rshunt_mOhm * Gain) = Vadc_mV * 2
 */
#define BSP_ADC_INA180_GAIN       50U
#define BSP_ADC_SHUNT_MOHM        10U

/* 12V/6.5V：90% 标称电压。24V/19V 到位不看电流。 */
#define BSP_ADC_GOOD_12V_MV      10800U
#define BSP_ADC_GOOD_6V5_MV       5850U

typedef struct
{
  pwr_rail_id_t rail;
  uint8_t buf_idx;
  uint32_t scale_num;
  uint32_t scale_den;
  uint32_t good_min_mv;
} adc_volt_map_t;

typedef struct
{
  pwr_rail_id_t rail;
  uint8_t buf_idx;
} adc_curr_map_t;

extern ADC_HandleTypeDef hadc1;

/*
 * GPDMA 配置为半字传输（Src/Dest = HALFWORD，dest 每次 +2）。
 * 必须用 uint16_t[9]：若用 uint32_t[9]，会读到错位的 Rank。
 */
static uint16_t s_adc_dma_buf[BSP_ADC_CHANNEL_COUNT] __attribute__((aligned(4)));
static adc_rails_status_t s_adc_status;

static const adc_volt_map_t s_volt_map[] =
{
  { PWR_RAIL_12V, BSP_ADC_IDX_12V_V, BSP_ADC_DIV_12_NUM, BSP_ADC_DIV_12_DEN,
    BSP_ADC_GOOD_12V_MV },
  { PWR_RAIL_6V5, BSP_ADC_IDX_7V5_V, BSP_ADC_DIV_6V5_NUM, BSP_ADC_DIV_6V5_DEN,
    BSP_ADC_GOOD_6V5_MV },
};

static const adc_curr_map_t s_curr_map[] =
{
  { PWR_RAIL_12V, BSP_ADC_IDX_12V_I },
  { PWR_RAIL_19V, BSP_ADC_IDX_19V_I },
  { PWR_RAIL_24V, BSP_ADC_IDX_24V_I },
  { PWR_RAIL_5V,  BSP_ADC_IDX_5V_I  },
  { PWR_RAIL_6V5, BSP_ADC_IDX_7V5_I },
};

static uint16_t AdcRails_ReadRaw(uint8_t buf_idx)
{
  if (buf_idx >= BSP_ADC_CHANNEL_COUNT)
  {
    return 0U;
  }

  return (uint16_t)(s_adc_dma_buf[buf_idx] & 0x0FFFU);
}

static uint32_t AdcRails_RawToRailMv(uint16_t raw, uint32_t scale_num,
                                     uint32_t scale_den)
{
  uint64_t num;

  if ((scale_den == 0U) || (raw == 0U))
  {
    return 0U;
  }

  num = (uint64_t)raw * (uint64_t)BSP_ADC_VREF_MV * (uint64_t)scale_num;
  return (uint32_t)(num / ((uint64_t)BSP_ADC_MAX_RAW * (uint64_t)scale_den));
}

static uint32_t AdcRails_RawToShuntCurrentMa(uint16_t raw)
{
  uint32_t den = BSP_ADC_SHUNT_MOHM * BSP_ADC_INA180_GAIN;
  uint64_t num;
  uint64_t div;

  if (den == 0U)
  {
    return 0U;
  }
  /*根据分压比计算电压 I=(raw*VREF*1000)/(ADC_MAX*Rshunt*Gain)*/
  div = (uint64_t)BSP_ADC_MAX_RAW * (uint64_t)den;
  num = (uint64_t)raw * (uint64_t)BSP_ADC_VREF_MV * 1000ULL;
  return (uint32_t)((num + (div / 2ULL)) / div);//四舍五入
}

static void AdcRails_RefreshStatus(void)
{
  uint8_t ch;
  uint32_t i;

  if (!s_adc_status.ready)
  {
    return;
  }

  for (ch = 0U; ch < BSP_ADC_CHANNEL_COUNT; ch++)
  {
    s_adc_status.channel_raw[ch] = AdcRails_ReadRaw(ch);
  }

  for (i = 0U; i < (uint32_t)PWR_RAIL_COUNT; i++)
  {
    s_adc_status.rail_mv[i] = 0U;
    s_adc_status.rail_ma[i] = 0U;
  }
  /*读取电压*/
  for (i = 0U; i < (sizeof(s_volt_map) / sizeof(s_volt_map[0])); i++)
  {
    const adc_volt_map_t *map = &s_volt_map[i];
    uint16_t raw = s_adc_status.channel_raw[map->buf_idx];

    s_adc_status.rail_mv[map->rail] =
        AdcRails_RawToRailMv(raw, map->scale_num, map->scale_den);
  }
  /*读取电流*/
  for (i = 0U; i < (sizeof(s_curr_map) / sizeof(s_curr_map[0])); i++)
  {
    const adc_curr_map_t *map = &s_curr_map[i];
    uint16_t raw = s_adc_status.channel_raw[map->buf_idx];

    s_adc_status.rail_ma[map->rail] = AdcRails_RawToShuntCurrentMa(raw);
  }
}

bool BSP_AdcRails_Init(void)
{
  if (s_adc_status.ready)
  {
    return true;
  }

  (void)memset(&s_adc_status, 0, sizeof(s_adc_status));

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return false;
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(void *)s_adc_dma_buf,
                        BSP_ADC_CHANNEL_COUNT) != HAL_OK)
  {
    return false;
  }

  /* 等第一轮 DMA 扫描完成（main 里调用，调度器尚未启动） */
  HAL_Delay(20U);
  s_adc_status.ready = true;
  AdcRails_RefreshStatus();
  return true;
}

bool BSP_AdcRails_IsReady(void)
{
  return s_adc_status.ready;
}

void BSP_AdcRails_Update(void)
{
  AdcRails_RefreshStatus();
}

const adc_rails_status_t *BSP_AdcRails_GetStatus(void)
{
  return &s_adc_status;
}

uint32_t BSP_AdcRails_GetRailMv(pwr_rail_id_t rail)
{
  if (!s_adc_status.ready || (rail >= PWR_RAIL_COUNT) ||
      (rail == PWR_RAIL_19V) || (rail == PWR_RAIL_24V) ||
      (rail == PWR_RAIL_5V))
  {
    return 0U;
  }

  return s_adc_status.rail_mv[rail];
}

uint32_t BSP_AdcRails_GetRailMa(pwr_rail_id_t rail)
{
  if (!s_adc_status.ready || (rail >= PWR_RAIL_COUNT))
  {
    return 0U;
  }

  return s_adc_status.rail_ma[rail];
}

bool BSP_AdcRails_IsRailGood(pwr_rail_id_t rail)
{
  uint32_t i;

  AdcRails_RefreshStatus();

  if (!s_adc_status.ready)
  {
    return false;
  }

  /* 12V / 6.5V：有分压，用电压判断。 */
  for (i = 0U; i < (sizeof(s_volt_map) / sizeof(s_volt_map[0])); i++)
  {
    if (s_volt_map[i].rail == rail)
    {
      return s_adc_status.rail_mv[rail] >= s_volt_map[i].good_min_mv;
    }
  }

  return false;
}
