/**
 ******************************************************************************
 * @file    charge_manager.h
 * @brief   6S 充电状态机：启动 / CC-CV / 停止 / 满电退出。
 *
 * 对外状态：充电中、已完成、异常（空闲为内部态，未启动充电）。
 * 通过 charge_path 控制 CFETOFF；BQ76942 CHG FET 由本模块使能。
 * 周期调用 ChargeManager_Process()（建议 500 ms，与 BmsTask 一致）。
 ******************************************************************************
 */
#ifndef CHARGE_MANAGER_H
#define CHARGE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32u3xx_hal.h"
#include "charge_gate.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 对外充电状态 */
typedef enum
{
  CHARGE_STATE_IDLE = 0,      /* 未充电（内部） */
  CHARGE_STATE_CHARGING,      /* 充电中 */
  CHARGE_STATE_COMPLETED,     /* 已完成（满电退出） */
  CHARGE_STATE_FAULT          /* 异常 */
} charge_state_t;

/** CC/CV 子阶段（仅 CHARGING 时有效） */
typedef enum
{
  CHARGE_PHASE_NONE = 0,
  CHARGE_PHASE_CC, /*恒流充电*/
  CHARGE_PHASE_CV /*恒压充电*/
} charge_phase_t;

typedef enum
{
  CHARGE_FAULT_NONE = 0,
  CHARGE_FAULT_THERMAL,
  CHARGE_FAULT_IMBALANCE,
  CHARGE_FAULT_OVERVOLT,
  CHARGE_FAULT_UNDERVOLT,
  CHARGE_FAULT_OVERCURRENT,
  CHARGE_FAULT_COMM,
  CHARGE_FAULT_BQ_PROTECT,
  CHARGE_FAULT_USER_STOP,
  CHARGE_FAULT_TIMEOUT,
  CHARGE_FAULT_NO_CURRENT
} charge_fault_reason_t;

typedef struct
{
  charge_state_t state;
  charge_phase_t phase;
  charge_fault_reason_t fault_reason;

  bool user_start_request;    /* 用户已请求充电 */
  bool chg_fet_on;
  bool charge_allowed;        /* CFET 路径未被 thermal/imbalance/manager 阻断 */
  bool charge_paused;         /* 充电中但被 thermal/imbalance 暂停 */

  uint16_t vcell_min_mv;
  uint16_t vcell_max_mv;
  uint32_t pack_mv;
  int16_t pack_current_ma;

  uint8_t cv_taper_count;     /* CV 涓流确认计数 */
  uint32_t charge_elapsed_ms;
  charge_reject_t last_reject;
  uint16_t last_reject_mask;
} charge_status_t;

void ChargeManager_Init(void);

/** 请求启动充电；IDLE/COMPLETED(需电压回落)；可恢复 FAULT 会先清再启。 */
bool ChargeManager_Start(void);

/**
 * @param require_charger true = 底板/ROS 开充，需充电桩 LIN 已就绪
 */
bool ChargeManager_RequestStart(bool require_charger);

const charge_gate_result_t *ChargeManager_GetLastReject(void);
void ChargeManager_ClearLastReject(void);
void ChargeManager_SetLastReject(const charge_gate_result_t *gate);

/** 用户停止充电，回到 IDLE。 */
void ChargeManager_Stop(void);

/** 清除可恢复异常后回到 IDLE（需温度/通信/保护已恢复）。 */
bool ChargeManager_ClearFault(void);

/** After max Δ-pause cycles: COMPLETED if taper full, else FAULT_IMBALANCE. */
void ChargeManager_FinishAfterImbalanceCycles(void);

/** LIN 会话 ACTIVE 时电流由充电桩提供，不要因尚未出流报 NO_CURRENT。 */
void ChargeManager_SetLinChargeExpect(bool expect);

/** 周期处理：采样、CC/CV 判定、FET/CFETOFF 执行。 */
void ChargeManager_Process(I2C_HandleTypeDef *hi2c);

const charge_status_t *ChargeManager_GetStatus(void);
charge_state_t ChargeManager_GetState(void);
charge_phase_t ChargeManager_GetPhase(void);

#ifdef __cplusplus
}
#endif

#endif /* CHARGE_MANAGER_H */
