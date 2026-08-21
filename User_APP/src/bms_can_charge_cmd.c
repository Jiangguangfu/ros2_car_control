/**
 * @file    bms_can_charge_cmd.c
 * @brief   底板 → BMS 充电命令（0x4A0）与仲裁应答（0x4A1）
 */
#include "bms_can_charge_cmd.h"

#include "can_uart_transport.h"
#include "charge_gate.h"
#include "charge_manager.h"
#include "main.h"

#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

#define BMS_CAN_CHG_CMD_STOP    0u
#define BMS_CAN_CHG_CMD_START   1u
#define BMS_CAN_CHG_CMD_QUERY   2u

#define BMS_CAN_CHG_FLAG_ACCEPTED  (1u << 0)
#define BMS_CAN_CHG_FLAG_CHARGING  (1u << 1)
#define BMS_CAN_CHG_FLAG_PAUSED    (1u << 2)

#define BMS_CAN_TX_FIFO_WAIT_MS  10U

static uint8_t s_rx_data[8];
static volatile bool s_rx_pending;

void BMS_CanChargeCmd_OnRx(const uint8_t *data)
{
  if (data == NULL)
  {
    return;
  }

  (void)memcpy(s_rx_data, data, 8U);
  s_rx_pending = true;
}

void BMS_CanChargeCmd_ApplyFilters(void)
{
  FDCAN_FilterTypeDef flt;

  if (hfdcan1.Instance == NULL)
  {
    return;
  }

  (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  flt.IdType = FDCAN_STANDARD_ID;
  flt.FilterType = FDCAN_FILTER_MASK;
  flt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  flt.FilterID2 = 0x7FF;

  flt.FilterIndex = 0;
  flt.FilterID1 = CAN_UART_ID_CHARGE_CTRL;
  (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &flt);

  flt.FilterIndex = 1;
  flt.FilterID1 = CAN_UART_ID_CHARGE_CMD;
  (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &flt);
}

static bool BMS_CanChargeCmd_WaitFifoFree(void)
{
  uint32_t t0 = HAL_GetTick();

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 1U)
  {
    if ((HAL_GetTick() - t0) >= BMS_CAN_TX_FIFO_WAIT_MS)
    {
      return false;
    }
  }

  return true;
}

static void BMS_CanChargeCmd_SendRsp(uint8_t cmd, uint16_t request_id,
                                     bool accepted)
{
  FDCAN_TxHeaderTypeDef hdr;
  uint8_t data[8];
  const charge_status_t *chg = ChargeManager_GetStatus();
  const charge_gate_result_t *rej = ChargeManager_GetLastReject();
  uint8_t flags = 0U;
  bool charging = false;
  bool paused = false;

  if (chg != NULL)
  {
    paused = chg->charge_paused;
    charging = (chg->state == CHARGE_STATE_CHARGING) && (!paused) &&
               chg->charge_allowed;
  }

  if (accepted)
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_ACCEPTED);
  }
  if (charging)
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_CHARGING);
  }
  if (paused)
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_PAUSED);
  }

  (void)memset(data, 0, sizeof(data));
  data[0] = cmd;
  data[1] = flags;
  data[2] = (uint8_t)(request_id & 0xFFU);
  data[3] = (uint8_t)((request_id >> 8) & 0xFFU);
  data[4] = (uint8_t)((rej != NULL) ? rej->code : CHARGE_REJECT_NONE);
  data[5] = (chg != NULL) ? (uint8_t)chg->state : 0U;
  if (rej != NULL)
  {
    data[6] = (uint8_t)(rej->mask & 0xFFU);
    data[7] = (uint8_t)((rej->mask >> 8) & 0xFFU);
  }

  if (accepted)
  {
    data[4] = (uint8_t)CHARGE_REJECT_NONE;
  }

  if (!BMS_CanChargeCmd_WaitFifoFree())
  {
    return;
  }

  hdr.Identifier = CAN_UART_ID_CHARGE_RSP;
  hdr.IdType = FDCAN_STANDARD_ID;
  hdr.TxFrameType = FDCAN_DATA_FRAME;
  hdr.DataLength = FDCAN_DLC_BYTES_8;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch = FDCAN_BRS_OFF;
  hdr.FDFormat = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker = 0;
  (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, data);
}

static void BMS_CanChargeCmd_Handle(const uint8_t *data)
{
  uint8_t cmd;
  uint16_t request_id;
  bool accepted = false;
  charge_gate_result_t gate;

  cmd = data[0];
  request_id = (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));

  if (cmd == BMS_CAN_CHG_CMD_STOP)
  {
    ChargeManager_Stop();
    ChargeManager_ClearLastReject();
    accepted = true;
  }
  else if (cmd == BMS_CAN_CHG_CMD_START)
  {
    accepted = ChargeManager_RequestStart(true);
  }
  else if (cmd == BMS_CAN_CHG_CMD_QUERY)
  {
    ChargeGate_Evaluate(true, &gate);
    ChargeManager_SetLastReject(&gate);
    accepted = (gate.code == CHARGE_REJECT_NONE);
  }
  else
  {
    accepted = false;
  }

  BMS_CanChargeCmd_SendRsp(cmd, request_id, accepted);
}

void BMS_CanChargeCmd_Init(void)
{
  BMS_CanChargeCmd_ApplyFilters();
}

void BMS_CanChargeCmd_Process(void)
{
  uint8_t data[8];

  if (!s_rx_pending)
  {
    return;
  }

  (void)memcpy(data, s_rx_data, 8U);
  s_rx_pending = false;
  BMS_CanChargeCmd_Handle(data);
}