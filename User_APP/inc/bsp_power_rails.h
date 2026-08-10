/**
 ******************************************************************************
 * @file    bsp_power_rails.h
 * @brief   Multi-rail power output management (24/19/12/6.5/5 V).
 *
 * GPIO mapping (active-high enable):
 *   24V  — PC13 PWR_24V_BYPASS_EN
 *   19V  — PB4  PWR_19V_EN
 *   12V  — PA8  PER_12V_EN
 *   6.5V — PB15 PWR_7V5_EN (hardware 7V5 label)
 *   5V   — no dedicated EN; follows 6.5V bus LDO (software-linked)
 *
 * Sources AND into final enable mask:
 *   - thermal manager request
 *   - OC/SC protect policy (BQ SCD/OCD/OCC + soft discharge OC)
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

typedef enum
{
  PWR_REQ_THERMAL = 0,
  PWR_REQ_PROTECT,
  PWR_REQ_COUNT
} pwr_req_source_t;

#define PWR_MASK_24V   (1u << PWR_RAIL_24V)
#define PWR_MASK_19V   (1u << PWR_RAIL_19V)
#define PWR_MASK_12V   (1u << PWR_RAIL_12V)
#define PWR_MASK_6V5   (1u << PWR_RAIL_6V5)
#define PWR_MASK_5V    (1u << PWR_RAIL_5V)
#define PWR_MASK_ALL   (PWR_MASK_24V | PWR_MASK_19V | PWR_MASK_12V | \
                        PWR_MASK_6V5 | PWR_MASK_5V)

typedef struct
{
  bool rail_on[PWR_RAIL_COUNT];
  uint8_t enabled_mask;
  uint8_t request_mask[PWR_REQ_COUNT];
} pwr_rails_status_t;

typedef enum
{
  PROTECT_STATE_NORMAL = 0,
  PROTECT_STATE_WARN,
  PROTECT_STATE_FAULT
} protect_state_t;

typedef enum
{
  PROTECT_REASON_NONE = 0,
  PROTECT_REASON_SCD,
  PROTECT_REASON_OCD,
  PROTECT_REASON_OCC,
  PROTECT_REASON_SOFT_OCD
} protect_reason_t;

typedef struct
{
  protect_state_t state;
  protect_reason_t reason;
  uint8_t power_rails_mask;
  bool charge_inhibit;
  bool discharge_inhibit;
  bool latched;
  bool safety_ok;
  uint8_t status_a;
  int16_t pack_current_ma;
} protect_status_t;

void BSP_PowerRails_Init(void);
void BSP_PowerRails_BootSequence(void);
void BSP_PowerRails_SetRequest(pwr_req_source_t source, uint8_t enable_mask);
void BSP_PowerRails_Apply(void);
void BSP_PowerRails_ApplyMask(uint8_t enable_mask);
const pwr_rails_status_t *BSP_PowerRails_GetStatus(void);

/** OC/SC policy → rails + charge_path inhibit. Call from PowerTask. */
void Protect_Init(void);
void Protect_Process(void);
const protect_status_t *Protect_GetStatus(void);
protect_state_t Protect_GetState(void);
bool Protect_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_POWER_RAILS_H */
