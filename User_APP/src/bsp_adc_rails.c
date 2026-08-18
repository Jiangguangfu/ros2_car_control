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

/* DMA Rank 索引（0-based，与 MX_ADC1_Init Rank 顺序一致）
 * 12V/24V 按实测对调：PA0 raw≈3176 → 12V≈11.1V；PA2 raw=0 → 24V 关闭。
 */
#define BSP_ADC_IDX_12V           0U  /* PA0 IN3 12V 分压 100k/30k */
#define BSP_ADC_IDX_19V           1U  /* PA1 IN4 19V 分压 100k/20k */
#define BSP_ADC_IDX_24V           2U  /* PA2 IN5 24V 分压 200k/20k */
#define BSP_ADC_IDX_5V            3U  /* PA3 IN6 ADC_M_5V 电流     */
#define BSP_ADC_IDX_7V5           8U  /* PB2 IN15 Rank9：实测 raw≈2046，不是 PA6 */

/*
 * 电压：Vrail_mV = raw * VREF * (Rtop + Rbot) / (ADC_MAX * Rbot)
 *
 *   12V  PA0 Rank1  Rtop=100k Rbot=30k  → 13/3
 *   19V  PA1 Rank2  Rtop=100k Rbot=20k  → 6/1
 *   24V  PA2 Rank3  Rtop=200k Rbot=20k  → 11/1
 *   6.5V PB2 Rank9  丝印在 PA6，实测 PA6=0、Rank9 raw≈2046、Vpin≈1.65V → 4/1
 */
#define BSP_ADC_DIV_24_NUM        11U
#define BSP_ADC_DIV_24_DEN         1U
#define BSP_ADC_DIV_19_NUM         6U
#define BSP_ADC_DIV_19_DEN         1U
#define BSP_ADC_DIV_12_NUM        13U
#define BSP_ADC_DIV_12_DEN         3U
#define BSP_ADC_DIV_6V5_NUM        4U
#define BSP_ADC_DIV_6V5_DEN        1U

/*
 * 5V 电流：ADC_M_5V ← INA180A2IDBVT（Gain=50）× Rshunt=10 mΩ
 * I_mA = Vadc_mV * 1000 / (Rshunt_mOhm * Gain) = Vadc_mV * 2
 * 原理图 R4 标 "3A 10m-30m"，与 24V 侧 R7=10 mΩ 对齐，暂按 10 mΩ。
 */
#define BSP_ADC_INA180_GAIN       50U
#define BSP_ADC_5V_SHUNT_MOHM     10U

/* 90% 标称电压作为 Good 阈值（mV） */
#define BSP_ADC_GOOD_24V_MV      21600U
#define BSP_ADC_GOOD_19V_MV      17100U
#define BSP_ADC_GOOD_12V_MV      10800U
#define BSP_ADC_GOOD_6V5_MV       5850U

typedef struct
{
  pwr_rail_id_t rail;
  uint8_t buf_idx;
  uint32_t scale_num;
  uint32_t scale_den;
  uint32_t good_min_mv;
} adc_rail_map_t;

extern ADC_HandleTypeDef hadc1;

/*
 * GPDMA 配置为半字传输（Src/Dest = HALFWORD，dest 每次 +2）。
 * 必须用 uint16_t[9]：若用 uint32_t[9]，19V/6.5V 会读到错位的 Rank。
 */
static uint16_t s_adc_dma_buf[BSP_ADC_CHANNEL_COUNT] __attribute__((aligned(4)));
static adc_rails_status_t s_adc_status;

static const adc_rail_map_t s_rail_map[] =
{
  /* 本项目不开 24V，仍换算电压供监测；不参与 WaitRailGood */
  { PWR_RAIL_24V, BSP_ADC_IDX_24V, BSP_ADC_DIV_24_NUM, BSP_ADC_DIV_24_DEN, 0U },
  { PWR_RAIL_19V, BSP_ADC_IDX_19V, BSP_ADC_DIV_19_NUM, BSP_ADC_DIV_19_DEN,
    BSP_ADC_GOOD_19V_MV },
  { PWR_RAIL_12V, BSP_ADC_IDX_12V, BSP_ADC_DIV_12_NUM, BSP_ADC_DIV_12_DEN,
    BSP_ADC_GOOD_12V_MV },
  { PWR_RAIL_6V5, BSP_ADC_IDX_7V5, BSP_ADC_DIV_6V5_NUM, BSP_ADC_DIV_6V5_DEN,
    BSP_ADC_GOOD_6V5_MV },
};

static const adc_rail_map_t *AdcRails_FindMap(pwr_rail_id_t rail)
{
  for (uint32_t i = 0U; i < (sizeof(s_rail_map) / sizeof(s_rail_map[0])); i++)
  {
    if (s_rail_map[i].rail == rail)
    {
      return &s_rail_map[i];
    }
  }

  return NULL;
}

static uint16_t AdcRails_ReadRaw(uint8_t buf_idx)
{
  if (buf_idx >= BSP_ADC_CHANNEL_COUNT)
  {
    return 0U;
  }

  return (uint16_t)(s_adc_dma_buf[buf_idx] & 0x0FFFU);
}

static uint32_t AdcRails_RawToPinMv(uint16_t raw)
{
  return (uint32_t)(((uint64_t)raw * (uint64_t)BSP_ADC_VREF_MV) /
                    (uint64_t)BSP_ADC_MAX_RAW);
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

static uint32_t AdcRails_RawTo5VCurrentMa(uint16_t raw)
{
  uint32_t pin_mv = AdcRails_RawToPinMv(raw);
  uint32_t den = BSP_ADC_5V_SHUNT_MOHM * BSP_ADC_INA180_GAIN;

  if (den == 0U)
  {
    return 0U;
  }

  return (pin_mv * 1000U) / den;
}

static void AdcRails_RefreshStatus(void)
{
  uint8_t ch;

  if (!s_adc_status.ready)
  {
    return;
  }

  for (ch = 0U; ch < BSP_ADC_CHANNEL_COUNT; ch++)
  {
    s_adc_status.channel_raw[ch] = AdcRails_ReadRaw(ch);
  }

  for (uint8_t i = 0U; i < (uint8_t)PWR_RAIL_COUNT; i++)
  {
    s_adc_status.rail_mv[i] = 0U;
  }

  for (uint32_t i = 0U; i < (sizeof(s_rail_map) / sizeof(s_rail_map[0])); i++)
  {
    const adc_rail_map_t *map = &s_rail_map[i];
    uint16_t raw = s_adc_status.channel_raw[map->buf_idx];

    s_adc_status.rail_mv[map->rail] =
        AdcRails_RawToRailMv(raw, map->scale_num, map->scale_den);
  }

  s_adc_status.i5v_ma =
      AdcRails_RawTo5VCurrentMa(s_adc_status.channel_raw[BSP_ADC_IDX_5V]);
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
  if (!s_adc_status.ready || (rail >= PWR_RAIL_COUNT))
  {
    return 0U;
  }

  return s_adc_status.rail_mv[rail];
}

bool BSP_AdcRails_IsRailGood(pwr_rail_id_t rail)
{
  const adc_rail_map_t *map = AdcRails_FindMap(rail);
  uint32_t mv;

  AdcRails_RefreshStatus();

  if (!s_adc_status.ready || (map == NULL) || (map->good_min_mv == 0U))
  {
    return false;
  }

  mv = s_adc_status.rail_mv[rail];
  return mv >= map->good_min_mv;
}
