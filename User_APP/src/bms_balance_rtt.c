/**
 * @file    bms_balance_rtt.c
 * @brief   均衡监控 RTT 文本（与 0x49B 同一快照，1 Hz 或 state/mask 变化）
 */
#include "bms_balance_rtt.h"

#include "bms_balance_snapshot.h"
#include "main.h"
#include "SEGGER_RTT.h"
#include "uart_battery_balance_report.h"

#include <stdbool.h>

#ifndef BMS_BALANCE_RTT_PERIOD_MS
#define BMS_BALANCE_RTT_PERIOD_MS     1000U
#endif

static uint32_t s_last_ms;
static uint8_t s_last_state;
static uint16_t s_last_mask;

static const char *bal_state_name(uint8_t state)
{
  switch (state)
  {
    case 0U: return "DIS";
    case 1U: return "IDLE";
    case 2U: return "WAIT";
    case 3U: return "TOP";
    case 4U: return "INH";
    case 5U: return "MREL";
    case 6U: return "MPRO";
    default: return "?";
  }
}

static const char *bal_phase_name(uint8_t flags, uint8_t state)
{
  if ((flags & BMS_BAL_FLAG_TOP_WINDOW) != 0U)
  {
    return "top";
  }

  if ((state == 5U) || (state == 6U))
  {
    return "mid";
  }

  return "off";
}

void BmsBalanceRtt_Init(void)
{
  s_last_ms = 0U;
  s_last_state = 0xFFU;
  s_last_mask = 0xFFFFU;
}

void BmsBalanceRtt_Process(void)
{
  uart_battery_balance_report_t report;
  uint32_t now;
  bool periodic;
  bool changed;

  BmsBalanceSnapshot_Fill(&report);
  now = HAL_GetTick();
  periodic = ((now - s_last_ms) >= BMS_BALANCE_RTT_PERIOD_MS);
  changed = (report.state != s_last_state) ||
            (report.active_mask != s_last_mask);

  if (!periodic && !changed)
  {
    return;
  }

  (void)SEGGER_RTT_printf(
      0,
      "BAL %s %s d=%u msk=0x%02X inh=%u mid=%u f=0x%02X%s%s%s\r\n",
      bal_phase_name(report.flags, report.state),
      bal_state_name(report.state),
      (unsigned)report.delta_mv,
      (unsigned)report.active_mask,
      (unsigned)report.inhibit_reason,
      (unsigned)report.mid_class,
      (unsigned)report.flags,
      ((report.flags & BMS_BAL_FLAG_BLEEDING) != 0U) ? " BLEED" : "",
      ((report.flags & BMS_BAL_FLAG_DELTA_OK) != 0U) ? " dOK" : "",
      ((report.flags & BMS_BAL_FLAG_IMBALANCE_CHG) != 0U) ? " CHGoff" : "");

  s_last_ms = now;
  s_last_state = report.state;
  s_last_mask = report.active_mask;
}
