/**
 * @file    bms_can_tx.c
 * @brief   UART 0x8B → CAN 0x48B 分片发送（500 kbit/s，5 Hz）
 */
#include "bms_can_tx.h"

#include "bms_can_charge_cmd.h"
#include "bms_data_snapshot.h"
#include "can_uart_transport.h"
#include "main.h"
#include "uart_battery_report.h"

#include <stdbool.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

/** FDCAN TX FIFO 深度为 3，连发 4 帧需等待 FIFO 腾出空位 */
#define BMS_CAN_TX_FIFO_WAIT_MS       10U

static void BMS_CanTx_RecoverIfBusOff(void)
{
  if ((hfdcan1.Instance == NULL) ||
      ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U)) {
    return;
  }

  (void)HAL_FDCAN_Stop(&hfdcan1);
  BMS_CanChargeCmd_ApplyFilters();
  (void)HAL_FDCAN_Start(&hfdcan1);
}

static bool BMS_CanTx_WaitFifoFree(void)
{
  uint32_t t0 = HAL_GetTick();

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 1U) {
    if ((HAL_GetTick() - t0) >= BMS_CAN_TX_FIFO_WAIT_MS) {
      return false;
    }
  }

  return true;
}

static bool BMS_CanTx_SendFrame(uint8_t frag_idx, uint8_t frag_total,
                                const uint8_t *payload, uint16_t offset)
{
  FDCAN_TxHeaderTypeDef hdr;
  uint8_t data[8];

  if (hfdcan1.Instance == NULL) {
    return false;
  }

  if (!BMS_CanTx_WaitFifoFree()) {
    return false;
  }

  data[0] = frag_idx;
  data[1] = frag_total;
  (void)memcpy(&data[2], &payload[offset], CAN_UART_FRAG_DATA_BYTES);

  hdr.Identifier = CAN_UART_ID_BATTERY;
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

void BMS_CanTx_Init(void)
{
}

void BMS_CanTx_Process(void)
{
  uart_battery_state_report_t report;
  uint8_t payload[CAN_UART_BATTERY_FRAG_TOTAL * CAN_UART_FRAG_DATA_BYTES];
  uint8_t frag;

  if (hfdcan1.Instance == NULL) {
    return;
  }

  BMS_CanTx_RecoverIfBusOff();

  BmsDataSnapshot_Fill(&report);
  (void)memset(payload, 0, sizeof(payload));
  (void)memcpy(payload, &report, sizeof(report));

  for (frag = 0U; frag < CAN_UART_BATTERY_FRAG_TOTAL; frag++) {
    uint16_t offset = (uint16_t)frag * CAN_UART_FRAG_DATA_BYTES;

    if (!BMS_CanTx_SendFrame(frag, CAN_UART_BATTERY_FRAG_TOTAL,
                             payload, offset)) {
      break;
    }
  }
}
