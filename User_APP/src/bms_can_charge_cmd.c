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
#define BMS_CAN_CHG_FLAG_CHARGING  (1u << 1)  /* 已确认充电电流 */
#define BMS_CAN_CHG_FLAG_PAUSED    (1u << 2)
#define BMS_CAN_CHG_FLAG_WAITING   (1u << 3)  /* ACK 后等待出流 */
#define BMS_CAN_CHG_FLAG_NO_FLOW   (1u << 4)  /* 开关开了但没充上 */

#define BMS_CAN_TX_FIFO_WAIT_MS  10U

static uint8_t s_rx_data[8];
static volatile bool s_rx_pending;
static uint8_t s_last_cmd;
static uint16_t s_last_request_id;
static uint8_t s_last_pub_flags;
static uint8_t s_last_pub_reject;

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
  bool waiting = false;
  bool no_flow = false;

  if (chg != NULL)
  {
    paused = chg->charge_paused;
    charging = chg->current_confirmed;
    no_flow = chg->no_current_after_ack;
    waiting = (chg->user_start_request != false) &&
              (chg->state == CHARGE_STATE_CHARGING) &&
              (!charging) && (!no_flow) && (!paused);
    if (chg->user_start_request)
    {
      accepted = true;
    }
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
  if (waiting)
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_WAITING);
  }
  if (no_flow)
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_NO_FLOW);
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

  s_last_pub_flags = flags;
  s_last_pub_reject = data[4];

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

  s_last_cmd = cmd;
  s_last_request_id = request_id;
  BMS_CanChargeCmd_SendRsp(cmd, request_id, accepted);
}

void BMS_CanChargeCmd_Init(void)
{
  BMS_CanChargeCmd_ApplyFilters();
}

void BMS_CanChargeCmd_PublishStatus(void)
{
  const charge_status_t *chg = ChargeManager_GetStatus();
  const charge_gate_result_t *rej = ChargeManager_GetLastReject();
  uint8_t flags = 0U;
  uint8_t reject = 0U;

  if (chg == NULL)
  {
    return;
  }

  if (chg->user_start_request)
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_ACCEPTED);
  }
  if (chg->current_confirmed)
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_CHARGING);
  }
  if (chg->charge_paused)
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_PAUSED);
  }
  if (chg->user_start_request && (chg->state == CHARGE_STATE_CHARGING) &&
      (!chg->current_confirmed) && (!chg->no_current_after_ack) &&
      (!chg->charge_paused))
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_WAITING);
  }
  if (chg->no_current_after_ack)
  {
    flags = (uint8_t)(flags | BMS_CAN_CHG_FLAG_NO_FLOW);
  }
  if (rej != NULL)
  {
    reject = (uint8_t)rej->code;
  }

  if ((flags == s_last_pub_flags) && (reject == s_last_pub_reject))
  {
    return;
  }

  if (chg->user_start_request && (s_last_cmd != BMS_CAN_CHG_CMD_START))
  {
    s_last_cmd = BMS_CAN_CHG_CMD_START;
  }

  BMS_CanChargeCmd_SendRsp(s_last_cmd, s_last_request_id,
                           chg->user_start_request != false);
}

void BMS_CanChargeCmd_NotifyHost(void)
{
  s_last_pub_flags = 0xFFU;
  s_last_pub_reject = 0xFFU;
  BMS_CanChargeCmd_PublishStatus();
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