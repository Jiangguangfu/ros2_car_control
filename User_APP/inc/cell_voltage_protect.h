/**
 ******************************************************************************
 * @file    cell_voltage_protect.h
 * @brief   BQ76942 COV/CUV + software low-voltage warning (charge/dsg inhibit).
 *
 * 电芯电压明细见 bq76942_meas_t（Bms_GetBqMeasurements()），本模块只维护保护状态。
 ******************************************************************************
 */
#ifndef CELL_VOLTAGE_PROTECT_H
#define CELL_VOLTAGE_PROTECT_H

#include <stdint.h>
#include <stdbool.h>
#include "bq76942.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 单节最低电压低于该值 → 低电量预警（不禁充放）。 */
#ifndef VOLTPROT_LOW_WARN_MV
#define VOLTPROT_LOW_WARN_MV              3300U
#endif
#ifndef VOLTPROT_LOW_WARN_CLEAR_MV
#define VOLTPROT_LOW_WARN_CLEAR_MV        3450U
#endif
/** 进出预警连续确认次数（BmsTask 500 ms × 3 ≈ 1.5 s），避免负载抖动误报。 */
#ifndef VOLTPROT_LOW_WARN_DEBOUNCE
#define VOLTPROT_LOW_WARN_DEBOUNCE          3U
#endif
/** 低于该值视为断线/掉电采样，不当成低电量（避免断电瞬间误报警）。 */
#ifndef VOLTPROT_CELL_VALID_MIN_MV
#define VOLTPROT_CELL_VALID_MIN_MV         2500U
#endif

typedef enum
{
  VOLTPROT_STATE_NORMAL = 0,
  VOLTPROT_STATE_LOW_WARN,
  VOLTPROT_STATE_COV,
  VOLTPROT_STATE_CUV
} cell_voltage_protect_state_t;

void CellVoltageProtect_Init(void);

/**
 * @param status_a  BQ Safety Status A (COV/CUV bits).
 * @param bq_valid  Safety status read succeeded.
 * @param meas      Latest bq76942_meas_t (for low-voltage warning).
 */
void CellVoltageProtect_Process(uint8_t status_a, bool bq_valid,
                                const bq76942_meas_t *meas);

cell_voltage_protect_state_t CellVoltageProtect_GetState(void);
uint8_t CellVoltageProtect_GetStatusA(void);
bool CellVoltageProtect_IsCov(void);
bool CellVoltageProtect_IsCuv(void);
bool CellVoltageProtect_IsLowVoltageWarn(void);
bool CellVoltageProtect_IsChargeInhibited(void);
bool CellVoltageProtect_IsDischargeInhibited(void);
bool CellVoltageProtect_IsValid(void);

#ifdef __cplusplus
}
#endif

#endif /* CELL_VOLTAGE_PROTECT_H */
