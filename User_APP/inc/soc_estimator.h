/**
 ******************************************************************************
 * @file    soc_estimator.h
 * @brief   6S NMC SOC：库仑计 + 静置 OCV 校正。
 *
 * 标称容量见 BMS_NOMINAL_CAPACITY_MAH（按实际电芯标定）。
 * SOH：满充时学习实测满充容量 / 标称（见 SOH_CHARGE_* 宏）。
 * 周期调用 Soc_Process()，建议与 BmsTask 一致（500 ms）。
 ******************************************************************************
 */
#ifndef SOC_ESTIMATOR_H
#define SOC_ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>

#include "bq76942.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 标称容量 (mAh)，按实际电池包修改 */
#ifndef BMS_NOMINAL_CAPACITY_MAH
#define BMS_NOMINAL_CAPACITY_MAH        12000U
#endif

typedef struct
{
  uint8_t soc_percent;       /* 0~100 */
  bool valid;
  int32_t remaining_mah;     /* 估算剩余容量 */
  int32_t nominal_mah;       /* 标称容量 */
  uint8_t soh_percent;       /* 0~100，容量衰减 SOH */
  bool soh_valid;            /* 至少完成一次可信满充学习 */
} soc_status_t;

void Soc_Init(void);
void Soc_Process(const bq76942_meas_t *meas, uint32_t period_ms);

const soc_status_t *Soc_GetStatus(void);
uint8_t Soc_GetPercent(void);
bool Soc_IsValid(void);
uint8_t Soc_GetSohPercent(void);
bool Soc_IsSohValid(void);

#ifdef __cplusplus
}
#endif

#endif /* SOC_ESTIMATOR_H */
