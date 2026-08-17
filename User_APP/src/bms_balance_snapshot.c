/**
 * @file    bms_balance_snapshot.c
 * @brief   Compact balance monitor → uart_battery_balance_report_t（0x9B / CAN 0x49B）
 */
#include "bms_balance_snapshot.h"

#include "cell_balance_manager.h"

#include <stdbool.h>
#include <string.h>

void BmsBalanceSnapshot_Fill(uart_battery_balance_report_t *out)
{
  const balance_status_t *balance;
  uint8_t flags = 0U;
  bool in_top;
  bool bleeding;

  if (out == NULL) {
    return;
  }

  (void)memset(out, 0, sizeof(*out));

  balance = Balance_GetStatus();
  if (balance == NULL) {
    return;
  }

  in_top = balance->top_start_ready ||
           (balance->state == BALANCE_STATE_ACTIVE);
  bleeding = (balance->state == BALANCE_STATE_ACTIVE) ||
             (balance->state == BALANCE_STATE_MID_PROTECT);

  if (balance->user_enabled) {
    flags = (uint8_t)(flags | BMS_BAL_FLAG_ENABLED);
  }
  if (balance->delta_ok) {
    flags = (uint8_t)(flags | BMS_BAL_FLAG_DELTA_OK);
  }
  if (balance->imbalance_charge_inhibit) {
    flags = (uint8_t)(flags | BMS_BAL_FLAG_IMBALANCE_CHG);
  }
  if (in_top) {
    flags = (uint8_t)(flags | BMS_BAL_FLAG_TOP_WINDOW);
  }
  if (bleeding) {
    flags = (uint8_t)(flags | BMS_BAL_FLAG_BLEEDING);
  }

  out->flags = flags;
  out->state = (uint8_t)balance->state;
  out->inhibit_reason = (uint8_t)balance->inhibit_reason;
  out->mid_class = (uint8_t)balance->mid_class;
  out->delta_mv = balance->delta_mv;
  out->active_mask = balance->active_mask;
}
