/**
 ******************************************************************************
 * @file    bsp_power_rails.h
 * @brief   Multi-rail power + unified protect (thermal / BQ OC-SC / soft OCD).
 *
 * GPIO mapping (active-high enable):
 *   24V  — PC13 PWR_24V_BYPASS_EN（S1 预充阶段打开）
 *   19V  — PB4  PWR_19V_EN
 *   12V  — PA8  PER_12V_EN
 *   6.5V — PB15 PWR_7V5_EN (hardware 7V5 label)
 *   5V   — no dedicated EN; follows 6.5V bus LDO (software-linked)
 *
 * 注意：PDSG/DSG 打开后 PACK+ 约等于电池电压（6S ≈ 22–25 V），
 * 那是 12V/19V 的输入母线，不是 PC13 控制的 24V 输出。
 ******************************************************************************
 */
#ifndef BSP_POWER_RAILS_H
#define BSP_POWER_RAILS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  PWR_RAIL_24V = 0,
  PWR_RAIL_19V,
  PWR_RAIL_12V,
  PWR_RAIL_6V5,
  PWR_RAIL_5V,
  PWR_RAIL_COUNT
} pwr_rail_id_t;

#define PWR_MASK_24V   (1u << PWR_RAIL_24V)
#define PWR_MASK_19V   (1u << PWR_RAIL_19V)
#define PWR_MASK_12V   (1u << PWR_RAIL_12V)
#define PWR_MASK_6V5   (1u << PWR_RAIL_6V5)
#define PWR_MASK_5V    (1u << PWR_RAIL_5V)
#define PWR_MASK_ALL   (PWR_MASK_24V | PWR_MASK_19V | PWR_MASK_12V | PWR_MASK_6V5 | PWR_MASK_5V)

typedef enum
{
  PWR_STATE_NORMAL = 0,
  PWR_STATE_WARN,
  PWR_STATE_LIMIT,
  PWR_STATE_FAULT
} pwr_state_t;

typedef enum
{
  PWR_REASON_NONE = 0,
  PWR_REASON_HOT,
  PWR_REASON_COLD_CHARGE,
  PWR_REASON_SENSOR,
  PWR_REASON_SCD,/*放电短路安全警报*/
  PWR_REASON_OCD,/*放电过流安全警报*/
  PWR_REASON_OCC,/*充电过流安全警报*/
  PWR_REASON_SOFT_OCD/*软过流安全警报*/
} pwr_reason_t;

typedef struct
{
  /* Actuation */
  bool rail_on[PWR_RAIL_COUNT];
  uint8_t enabled_mask;
  uint8_t power_rails_mask;
  uint8_t fan_duty_percent;
  bool charge_inhibit;
  bool discharge_inhibit;

  /* Combined severity (thermal vs current protect). */
  pwr_state_t state;
  pwr_reason_t reason;
  bool latched;

  /* Thermal sensors */
  int16_t tmax_c_x10;
  int16_t tmin_c_x10;
  int16_t die_c_x10;
  bool sensor_ok;

  /* BQ Safety Status (raw A/B/C + parsed current faults). */
  uint8_t status_a;
  uint8_t status_b;
  uint8_t status_c;
  bool scd;
  bool ocd;
  bool occ;
  bool bq_any;
  bool bq_valid;

  int16_t pack_current_ma;
} pwr_rails_status_t;

void BSP_PowerRails_Init(void);
/** RTOS 前：全部电源轨关闭，供 ServiceTask 分步上电。 */
void BSP_PowerRails_PreBoot(void);
void BSP_PowerRails_BootSequence(void);
/** 逐步使能单路电源轨（更新 rail_on / enabled_mask）。 */
bool BSP_PowerRails_EnableRail(pwr_rail_id_t rail, bool on);
/** 等待电源轨就绪：12V/6.5V 看电压（连续 3 次），24V 看 S1 已拉高 EN，
 *  19V 看 EN 已拉高且无短路反馈，延时后认为已打开。 */
bool BSP_PowerRails_WaitRailGood(pwr_rail_id_t rail, uint32_t timeout_ms);
/** ServiceTask 上电序列完成后置 true，Init/Process 才接管轨控。 */
void BSP_PowerRails_SetBootComplete(bool complete);
bool BSP_PowerRails_IsBootComplete(void);

/** Cache Safety A/B/C from BmsTask and parse SCD/OCD/OCC. */
void BSP_PowerRails_UpdateBqSafety(uint8_t status_a, uint8_t status_b,
                                   uint8_t status_c, bool valid);

/** Evaluate thermal + OC/SC and drive rails / fan / FET. Call from PowerTask. */
void BSP_PowerRails_Process(void);

const pwr_rails_status_t *BSP_PowerRails_GetStatus(void);
pwr_state_t BSP_PowerRails_GetState(void);

/**
 * Optional: force one recover evaluation now.
 * Process() already auto-recovers: hot when cooled, OC/SC when flags/current clear.
 */
bool BSP_PowerRails_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_POWER_RAILS_H */
