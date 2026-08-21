/**
 * @file    bms_can_ext_tx.c
 * @brief   UART 0x9A → CAN 0x49A 扩展包（告警 + 明细，1 Hz 或告警变化即发）
 */
#include "bms_can_ext_tx.h"

#include "bms_can_charge_cmd.h"
#include "bms_ext_snapshot.h"
#include "can_uart_transport.h"
#include "main.h"
#include "uart_battery_ext_report.h"

#include <stdbool.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

#ifndef BMS_CAN_EXT_PERIOD_MS
#define BMS_CAN_EXT_PERIOD_MS         1000U
#endif

#define BMS_CAN_TX_FIFO_WAIT_MS       10U

static uint32_t s_last_tx_ms;
static uint32_t s_last_alarm_flags;

static void BMS_CanExtTx_RecoverIfBusOff(void)
{
  if ((hfdcan1.Instance == NULL) ||
      ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U)) {
    return;
  }

  (void)HAL_FDCAN_Stop(&hfdcan1);
  BMS_CanChargeCmd_ApplyFilters();
  (void)HAL_FDCAN_Start(&hfdcan1);
}

static bool BMS_CanExtTx_WaitFifoFree(void)
{
  uint32_t t0 = HAL_GetTick();

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 1U) {
    if ((HAL_GetTick() - t0) >= BMS_CAN_TX_FIFO_WAIT_MS) {
      return false;
    }
  }

  return true;
}

static bool BMS_CanExtTx_SendFrame(uint8_t frag_idx, uint8_t frag_total,
                                   const uint8_t *payload, uint16_t offset)
{
  FDCAN_TxHeaderTypeDef hdr;
  uint8_t data[8];

  if (hfdcan1.Instance == NULL) {
    return false;
  }

  if (!BMS_CanExtTx_WaitFifoFree()) {
    return false;
  }

  data[0] = frag_idx;
  data[1] = frag_total;
  (void)memcpy(&data[2], &payload[offset], CAN_UART_FRAG_DATA_BYTES);

  hdr.Identifier = CAN_UART_ID_BATTERY_EXT;
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

static void BMS_CanExtTx_SendPayload(const uint8_t *payload, uint8_t frag_total)
{
  uint8_t frag;

  for (frag = 0U; frag < frag_total; frag++) {
    uint16_t offset = (uint16_t)frag * CAN_UART_FRAG_DATA_BYTES;

    if (!BMS_CanExtTx_SendFrame(frag, frag_total, payload, offset)) {
      break;
    }
  }
}

void BMS_CanExtTx_Init(void)
{
  s_last_tx_ms = 0U;
  s_last_alarm_flags = 0U;
}

void BMS_CanExtTx_Process(void)
{
  uart_battery_ext_report_t report;
  uint8_t payload[CAN_UART_BATTERY_EXT_FRAG_TOTAL * CAN_UART_FRAG_DATA_BYTES];
  uint32_t now;
  bool periodic;
  bool alarm_changed;

  if (hfdcan1.Instance == NULL) {
    return;
  }

  BmsExtSnapshot_Fill(&report);
  now = HAL_GetTick();
  periodic = ((now - s_last_tx_ms) >= BMS_CAN_EXT_PERIOD_MS);
  alarm_changed = (report.alarm_flags != s_last_alarm_flags);

  if (!periodic && !alarm_changed) {
    return;
  }

  BMS_CanExtTx_RecoverIfBusOff();

  (void)memset(payload, 0, sizeof(payload));
  (void)memcpy(payload, &report, sizeof(report));
  BMS_CanExtTx_SendPayload(payload, (uint8_t)CAN_UART_BATTERY_EXT_FRAG_TOTAL);

  s_last_tx_ms = now;
  s_last_alarm_flags = report.alarm_flags;
}
