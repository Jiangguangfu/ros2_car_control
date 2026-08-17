/**
 * @file    bms_can_balance_tx.c
 * @brief   UART 0x9B → CAN 0x49B 均衡监控（1 Hz 或 state/mask 变化即发）
 */
#include "bms_can_balance_tx.h"

#include "bms_balance_snapshot.h"
#include "can_uart_transport.h"
#include "main.h"
#include "uart_battery_balance_report.h"

#include <stdbool.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

#ifndef BMS_CAN_BALANCE_PERIOD_MS
#define BMS_CAN_BALANCE_PERIOD_MS     1000U
#endif

#define BMS_CAN_TX_FIFO_WAIT_MS       10U

static uint32_t s_last_tx_ms;
static uint8_t s_last_state;
static uint16_t s_last_mask;

static void BMS_CanBalanceTx_RecoverIfBusOff(void)
{
  if ((hfdcan1.Instance == NULL) ||
      ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U)) {
    return;
  }

  (void)HAL_FDCAN_Stop(&hfdcan1);
  (void)HAL_FDCAN_Start(&hfdcan1);
  (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

static bool BMS_CanBalanceTx_WaitFifoFree(void)
{
  uint32_t t0 = HAL_GetTick();

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 1U) {
    if ((HAL_GetTick() - t0) >= BMS_CAN_TX_FIFO_WAIT_MS) {
      return false;
    }
  }

  return true;
}

static bool BMS_CanBalanceTx_SendFrame(uint8_t frag_idx, uint8_t frag_total,
                                       const uint8_t *payload, uint16_t offset)
{
  FDCAN_TxHeaderTypeDef hdr;
  uint8_t data[8];

  if (hfdcan1.Instance == NULL) {
    return false;
  }

  if (!BMS_CanBalanceTx_WaitFifoFree()) {
    return false;
  }

  data[0] = frag_idx;
  data[1] = frag_total;
  (void)memcpy(&data[2], &payload[offset], CAN_UART_FRAG_DATA_BYTES);

  hdr.Identifier = CAN_UART_ID_BATTERY_BALANCE;
  hdr.IdType = FDCAN_STANDARD_ID;
  hdr.TxFrameType = FDCAN_DATA_FRAME;
  hdr.DataLength = FDCAN_DLC_BYTES_8;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch = FDCAN_BRS_OFF;
  hdr.FDFormat = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker = 0;

  return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, data) == HAL_OK);
}

static void BMS_CanBalanceTx_SendPayload(const uint8_t *payload, uint8_t frag_total)
{
  uint8_t frag;

  for (frag = 0U; frag < frag_total; frag++) {
    uint16_t offset = (uint16_t)frag * CAN_UART_FRAG_DATA_BYTES;

    if (!BMS_CanBalanceTx_SendFrame(frag, frag_total, payload, offset)) {
      break;
    }
  }
}

void BMS_CanBalanceTx_Init(void)
{
  s_last_tx_ms = 0U;
  s_last_state = 0xFFU;
  s_last_mask = 0xFFFFU;
}

void BMS_CanBalanceTx_Process(void)
{
  uart_battery_balance_report_t report;
  uint8_t payload[CAN_UART_BATTERY_BALANCE_FRAG_TOTAL * CAN_UART_FRAG_DATA_BYTES];
  uint32_t now;
  bool periodic;
  bool changed;

  if (hfdcan1.Instance == NULL) {
    return;
  }

  BmsBalanceSnapshot_Fill(&report);
  now = HAL_GetTick();
  periodic = ((now - s_last_tx_ms) >= BMS_CAN_BALANCE_PERIOD_MS);
  changed = (report.state != s_last_state) ||
            (report.active_mask != s_last_mask);

  if (!periodic && !changed) {
    return;
  }

  BMS_CanBalanceTx_RecoverIfBusOff();

  (void)memset(payload, 0, sizeof(payload));
  (void)memcpy(payload, &report, sizeof(report));
  BMS_CanBalanceTx_SendPayload(payload, (uint8_t)CAN_UART_BATTERY_BALANCE_FRAG_TOTAL);

  s_last_tx_ms = now;
  s_last_state = report.state;
  s_last_mask = report.active_mask;
}
