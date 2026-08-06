/**
 ******************************************************************************
 * @file    soh_estimator.h
 * @brief   电池健康 (SOH) — 以永久容量保持率为核心。
 *
 * SOH 定义：
 *   soh_percent ≈ learned_capacity / 标称容量（自动满充放标定，EMA 慢更新）
 *
 * 不参与 SOH 的瞬时/可恢复量（单独 alarm 字段）：
 *   BQ 保护、热管理 WARN/LIMIT/FAULT、通信失败
 *
 * 一致性指标（consistency_soh）仅诊断，表示静置压差/弱节，不代表容量衰减。
 ******************************************************************************
 */
#ifndef SOH_ESTIMATOR_H
#define SOH_ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>

#include "bq76942.h"
#include "charge_manager.h"
#include "thermal_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  SOH_STATE_UNKNOWN = 0,
  SOH_STATE_HEALTHY,
  SOH_STATE_DEGRADED,
  SOH_STATE_WARNING,
  SOH_STATE_CRITICAL
} soh_state_t;

typedef enum
{
  SOH_CAP_LEARN_IDLE = 0,
  SOH_CAP_LEARN_DISCHARGE,
  SOH_CAP_LEARN_CHARGE
} soh_capacity_learn_phase_t;

/** 容量/SOH 相关慢性因子（诊断） */
typedef enum
{
  SOH_FACTOR_NONE = 0,
  SOH_FACTOR_CAPACITY = (1U << 0),
  SOH_FACTOR_IMBALANCE = (1U << 1),
  SOH_FACTOR_WEAK_CELL = (1U << 2)
} soh_factor_mask_t;

/** 瞬时告警（不参与 soh_percent） */
typedef enum
{
  SOH_ALARM_NONE = 0,
  SOH_ALARM_BQ_PROTECT = (1U << 0),
  SOH_ALARM_THERMAL = (1U << 1),
  SOH_ALARM_COMM = (1U << 2)
} soh_alarm_mask_t;

typedef struct
{
  const bq76942_meas_t *meas;
  const bq76942_temp_t *temp;
  const thermal_status_t *thermal;
  charge_state_t charge_state;
  bool bq_protect;
  uint32_t comm_fail_count;
} soh_inputs_t;

typedef struct
{
  soh_state_t state;
  uint8_t soh_percent;             /* 0~100，容量 SOH（永久） */
  bool valid;

  uint8_t capacity_soh;            /* 容量保持率；未标定前 100 */
  uint8_t consistency_soh;         /* 静置一致性，诊断用 */

  uint16_t delta_mv;
  uint16_t vmin_mv;
  uint16_t vmax_mv;
  uint16_t weak_cell_lag_mv;

  soh_capacity_learn_phase_t capacity_learn_phase;
  uint32_t capacity_learn_accum_mah;
  bool capacity_learned;

  uint32_t chronic_factors;        /* SOH_FACTOR_* */
  uint32_t alarm_flags;            /* SOH_ALARM_*，瞬时 */

  uint32_t cycle_count;
  uint32_t learned_capacity_mah;
} soh_status_t;

void Soh_Init(void);
void Soh_Process(const soh_inputs_t *inputs, uint32_t period_ms);

const soh_status_t *Soh_GetStatus(void);
uint8_t Soh_GetPercent(void);
soh_state_t Soh_GetState(void);
bool Soh_IsValid(void);

#ifdef __cplusplus
}
#endif

#endif /* SOH_ESTIMATOR_H */
