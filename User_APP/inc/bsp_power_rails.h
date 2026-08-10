/**
 ******************************************************************************
 * @file    bsp_power_rails.h
 * @brief   Board multi-rail enable: 24V / 19V / 12V / 6.5V / 5V (logical).
 *
 * GPIO mapping (active-high enable):
 *   24V  — PC13 PWR_24V_BYPASS_EN
 *   19V  — PB4  PWR_19V_EN
 *   12V  — PA8  PER_12V_EN
 *   6.5V — PB15 PWR_7V5_EN (hardware 7V5 label)
 *   5V   — no dedicated EN; follows 6.5V bus LDO (software-linked)
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
#define PWR_MASK_ALL   (PWR_MASK_24V | PWR_MASK_19V | PWR_MASK_12V | \
                        PWR_MASK_6V5 | PWR_MASK_5V)

typedef struct
{
  bool rail_on[PWR_RAIL_COUNT];
  uint8_t enabled_mask;
} pwr_rails_status_t;

void BSP_PowerRails_Init(void);

/** Power-on sequencing (call once from main before RTOS). */
void BSP_PowerRails_BootSequence(void);

/** Apply enable mask; 5V bit tracks 6.5V when no separate GPIO. */
void BSP_PowerRails_ApplyMask(uint8_t enable_mask);

const pwr_rails_status_t *BSP_PowerRails_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_POWER_RAILS_H */
