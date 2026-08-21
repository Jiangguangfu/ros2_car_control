/**
 * @file    bms_can_rx.c
 * @brief   FDCAN RX：接收 407 下发的充电控制（CAN 0x441）
 */
#include "bms_can_rx.h"

#include "bms_can_charge_cmd.h"
#include "can_uart_transport.h"
#include "lin_charger.h"
#include "main.h"
#include "uart_charge_ctrl.h"

#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

typedef struct
{
  volatile bool pending;
  uint8_t cmd;
  uint16_t i_ma;
} bms_can_rx_pending_t;

static bms_can_rx_pending_t s_pending;

static void bms_can_rx_queue(uint8_t cmd, uint16_t i_ma)
{
  s_pending.cmd = cmd;
  s_pending.i_ma = i_ma;
  s_pending.pending = true;
}

void BMS_CanRx_Init(void)
{
  FDCAN_FilterTypeDef filter = {0};

  if (hfdcan1.Instance == NULL)
  {
    return;
  }

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  /* HAL expects an unshifted 11-bit standard ID and mask. */
  filter.FilterID1 = CAN_UART_ID_CHARGE_CTRL;
  filter.FilterID2 = 0x7FFU;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    return;
  }

  filter.FilterIndex = 1U;
  filter.FilterID1 = CAN_UART_ID_CHARGE_CMD;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    return;
  }

  (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  (void)HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef hdr;
  uint8_t data[8];
  uart_charge_ctrl_cmd_t cmd_pkt;

  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1) ||
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK)
    {
      break;
    }

    if (hdr.IdType != FDCAN_STANDARD_ID)
    {
      continue;
    }

    if (hdr.Identifier == CAN_UART_ID_CHARGE_CMD)
    {
      BMS_CanChargeCmd_OnRx(data);
      continue;
    }

    if ((hdr.Identifier != CAN_UART_ID_CHARGE_CTRL) ||
        (data[1] != CAN_UART_CHARGE_CTRL_FRAG_TOTAL) ||
        (data[0] != 0U))
    {
      continue;
    }

    (void)memcpy(&cmd_pkt, &data[2], sizeof(cmd_pkt));
    if (((cmd_pkt.cmd == UART_CHARGE_CTRL_STOP) &&
         (cmd_pkt.i_target_ma != 0U)) ||
        (((cmd_pkt.cmd == UART_CHARGE_CTRL_SET_CURRENT) ||
          (cmd_pkt.cmd == UART_CHARGE_CTRL_START)) &&
         !uart_charge_current_is_valid(cmd_pkt.i_target_ma)) ||
        (cmd_pkt.cmd > UART_CHARGE_CTRL_STOP))
    {
      continue;
    }
    bms_can_rx_queue(cmd_pkt.cmd, cmd_pkt.i_target_ma);
  }
}

void BMS_CanRx_Process(void)
{
  uint8_t cmd;
  uint16_t i_ma;

  if (!s_pending.pending)
  {
    return;
  }

  cmd = s_pending.cmd;
  i_ma = s_pending.i_ma;
  s_pending.pending = false;
  LinCharger_ApplyCanCommand(cmd, i_ma);
}
