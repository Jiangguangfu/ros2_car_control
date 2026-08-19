/**
 ******************************************************************************
 * @file    cell_balance_manager.h
 * @brief   6S NMC passive cell balancing via BQ76942 CB_ACTIVE_CELLS.
 *
 * Top-of-charge fine balance (40/15 mV) plus mid-charge highest-cell
 * protect and IR-vs-real classify. Mid-charge does not chase alignment.
 * Call Balance_Process() from BmsTask (500 ms).
 ******************************************************************************
 */
#ifndef CELL_BALANCE_MANAGER_H
#define CELL_BALANCE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32u3xx_hal.h"
#include "bq76942.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMS_CELL_COUNT                    BQ76942_CELL_COUNT
#define BMS_BALANCE_MASK_VALID         ((uint16_t)((1U << BMS_CELL_COUNT) - 1U))

#define BALANCE_START_DELTA_MV           40U
#define BALANCE_STOP_DELTA_MV            15U

/* Top-window charge pause only (independent of balance 40/15).
 * Bench override: 300/250 so a ~283 mV pack can still top-balance.
 * Restore 50/30 for production. Mid-charge does not use these. */
#define CHARGE_IMBALANCE_STOP_DELTA_MV    300U
#define CHARGE_IMBALANCE_RESUME_DELTA_MV  250U

#define BALANCE_HARD_MIN_CELL_MV       3000U
#define BALANCE_NORMAL_MAX_CELL_MV     4200U

#define BALANCE_VMIN_START_MV            3900U
#define BALANCE_VMIN_HOLD_MV             3850U

#define BALANCE_SOC_START_PERCENT          90U
#define BALANCE_SOC_HOLD_PERCENT           88U

#define BALANCE_MAX_CELLS_AT_ONCE           2U

#define BALANCE_CELL_MV_MIN_VALID        2500U
#define BALANCE_CELL_MV_MAX_VALID        4300U
#define BALANCE_SAMPLE_STABLE_COUNT         3U

#define BALANCE_CHARGE_DEBOUNCE_COUNT       3U
#define BALANCE_DISCHARGE_RECOVER_MA        (-50)
#define BALANCE_DISCHARGE_EXIT_MA          (-100)
#define BALANCE_DISCHARGE_EXIT_DEBOUNCE     2U

#define BALANCE_CELL_TEMP_MIN_CX10            0
#define BALANCE_CELL_TEMP_MAX_CX10          450
#define BALANCE_DIE_TEMP_MAX_CX10           600

#define BALANCE_COMM_FAIL_THRESHOLD           3U

/* Mid-charge: protect highest cell + classify IR vs real. Not equalization. */
#define BALANCE_MID_VMAX_WARN_MV           4000U
#define BALANCE_MID_VMAX_DANGER_MV         4180U
#define BALANCE_MID_VMAX_SAFE_MV           4150U
#define BALANCE_MID_OBSERVE_DELTA_MV        150U
#define BALANCE_MID_FAKE_SHRINK_MV           50U
#define BALANCE_MID_REST_REAL_DELTA_MV       80U
#define BALANCE_MID_RELAX_COUNT               6U  /* ~3 s @ 500 ms */
#define BALANCE_MID_PROTECT_MAX_TICKS        60U  /* ~30 s bleed cap */
#define BALANCE_MID_PROTECT_COOLDOWN         10U  /* ~5 s before re-enter */
#define BALANCE_MID_PROTECT_NEAR_MV          20U

typedef enum
{
  BALANCE_STATE_DISABLED = 0,
  BALANCE_STATE_IDLE,
  BALANCE_STATE_WAIT_CHARGE,
  BALANCE_STATE_ACTIVE,
  BALANCE_STATE_INHIBITED,
  BALANCE_STATE_MID_RELAX,
  BALANCE_STATE_MID_PROTECT
} balance_state_t;

typedef enum
{
  BALANCE_INHIBIT_NONE = 0,
  BALANCE_INHIBIT_DISABLED,
  BALANCE_INHIBIT_NOT_CHARGING,
  BALANCE_INHIBIT_TOP_NOT_READY,
  BALANCE_INHIBIT_HARD_UNDERVOLT,
  BALANCE_INHIBIT_OVERVOLT,
  BALANCE_INHIBIT_DELTA_LOW,
  BALANCE_INHIBIT_THERMAL,
  BALANCE_INHIBIT_SAMPLE_UNSTABLE,
  BALANCE_INHIBIT_COMM,
  BALANCE_INHIBIT_BQ_PROTECT,
  BALANCE_INHIBIT_DISCHARGE,
  BALANCE_INHIBIT_MID_OBSERVE,
  BALANCE_INHIBIT_MID_PROTECT
} balance_inhibit_reason_t;

typedef enum
{
  BALANCE_MID_CLASS_NONE = 0,
  BALANCE_MID_CLASS_FAKE_IR,
  BALANCE_MID_CLASS_REAL_SAFE,
  BALANCE_MID_CLASS_REAL_THREAT
} balance_mid_class_t;

typedef struct
{
  bool user_enabled;
  balance_state_t state;
  balance_inhibit_reason_t inhibit_reason;

  bool soc_valid;
  uint8_t soc_percent;
  bool top_start_ready;
  bool top_hold_ready;

  bool charger_present;
  bool chg_fet_on;
  bool discharge_detected;
  bool charger_active;
  bool charger_active_stable;
  bool imbalance_charge_inhibit; /* mid relax/protect or top Δ pause */

  bool start_conditions_ok;
  bool hold_conditions_ok;
  bool delta_ok;

  uint16_t cell_mv[BMS_CELL_COUNT];
  uint16_t vmin_mv;
  uint16_t vmax_mv;
  uint16_t delta_mv;
  uint16_t stack_mv;
  int16_t pack_current_ma;
  int16_t tmax_cell_c_x10;
  int16_t tmin_cell_c_x10;
  int16_t die_c_x10;

  uint16_t active_mask;
  uint8_t stable_count;
  uint8_t charge_stable_count;
  uint8_t discharge_exit_count;

  balance_mid_class_t mid_class;
  bool mid_observe_done;
  uint16_t mid_delta_at_pause_mv;
  uint16_t mid_vmax_at_pause_mv;
  uint8_t mid_relax_count;
  uint8_t mid_protect_count;
  uint8_t mid_protect_cooldown;
} balance_status_t;

void Balance_Init(void);
void Balance_Process(I2C_HandleTypeDef *hi2c);

void Balance_SetEnabled(bool enable);
bool Balance_IsEnabled(void);

/** Optional: dedicated charger-detect GPIO/CAN (P1). If never called, inferred. */
void Balance_SetChargerPresent(bool present);

/** Optional: SOC from coulomb-counting module (P2). If invalid, vmin fallback. */
void Balance_SetSoc(uint8_t soc_percent, bool valid);

const balance_status_t *Balance_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* CELL_BALANCE_MANAGER_H */
