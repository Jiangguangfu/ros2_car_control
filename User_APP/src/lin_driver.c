/**
 * @file    lin_driver.c
 * @brief   USART1 LIN 从机：Break 检测 → 收 Header/Data → 应答 → lin_charger。
 */
#include "lin_driver.h"

#include <string.h>

#include "bms_lin_config.h"
#include "lin_charger.h"
#include "lin_charger_protocol.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

typedef enum
{
  LIN_DRV_IDLE = 0,
  LIN_DRV_RX_SYNC,
  LIN_DRV_RX_PID,
  LIN_DRV_RX_MASTER,
  LIN_DRV_TX_SLAVE
} lin_drv_state_t;

typedef struct
{
  uint8_t id6;
  uint8_t master_len;
  uint8_t slave_len;
} lin_frame_map_t;

static const lin_frame_map_t s_frame_map[] = {
    {0x10u, 2u, 4u}, /* 握手 */
    {0x20u, 6u, 6u}, /* V/I 协商 */
    {0x30u, 2u, 0u}, /* 充电控制 */
    {0x32u, 0u, 8u}, /* 状态轮询 */
    {0x3Fu, 1u, 0u}, /* 心跳 */
};

static lin_drv_state_t s_state;
static uint8_t s_rx_pid;
static uint8_t s_master_buf[9];
static uint8_t s_master_len;
static uint8_t s_master_rx_total;
static uint8_t s_master_idx;
static uint8_t s_slave_len;
static uint8_t s_rx_byte;
static uint8_t s_tx_buf[9];
static volatile bool s_inited;

static lin_driver_diag_t s_diag;

static const lin_frame_map_t *lin_find_frame(uint8_t id6)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(s_frame_map) / sizeof(s_frame_map[0])); i++)
  {
    if (s_frame_map[i].id6 == id6)
    {
      return &s_frame_map[i];
    }
  }

  return NULL;
}

static uint8_t lin_checksum_enhanced(uint8_t pid, const uint8_t *data, uint8_t len)
{
  uint16_t sum = pid;
  uint8_t i;

  for (i = 0U; i < len; i++)
  {
    sum = (uint16_t)(sum + data[i]);
    if (sum > 0xFFU)
    {
      sum = (uint16_t)((sum & 0xFFU) + 1U);
    }
  }

  return (uint8_t)(~sum);
}

static void lin_clear_uart_errors(void)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
  }
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE))
  {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
  }
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE))
  {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_NEF);
  }
}

static void lin_flush_rx(void)
{
  lin_clear_uart_errors();

  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
  {
    (void)READ_REG(huart1.Instance->RDR);
  }
}

#define LIN_DRV_IO_TIMEOUT_MS   10U

static bool lin_send_slave_frame(uint8_t payload_len)
{
  uint16_t tx_len = (uint16_t)(payload_len + 1U);

  s_state = LIN_DRV_TX_SLAVE;
  s_diag.rsp_tx++;

  if (HAL_UART_Transmit(&huart1, s_tx_buf, tx_len, LIN_DRV_IO_TIMEOUT_MS) != HAL_OK)
  {
    s_state = LIN_DRV_IDLE;
    return false;
  }

  s_state = LIN_DRV_IDLE;
  return true;
}

static void lin_arm_rx_byte(void)
{
  if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK)
  {
    s_state = LIN_DRV_IDLE;
  }
}

static void lin_arm_rx_master(void)
{
  if (HAL_UART_Receive_IT(&huart1, &s_master_buf[s_master_idx], 1U) != HAL_OK)
  {
    s_state = LIN_DRV_IDLE;
  }
}

static void lin_finish_master_rx(void)
{
  uint8_t rsp[8];
  uint8_t rsp_len = 0U;
  const lin_frame_map_t *frame;
  uint8_t cs;

  frame = lin_find_frame((uint8_t)(s_rx_pid & 0x3Fu));
  if (frame == NULL)
  {
    s_state = LIN_DRV_IDLE;
    return;
  }

  if (s_master_len > 0U)
  {
    cs = lin_checksum_enhanced(s_rx_pid, s_master_buf, s_master_len);
    if (s_master_buf[s_master_len] != cs)
    {
      s_diag.cs_fail++;
      s_state = LIN_DRV_IDLE;
      return;
    }
  }

  (void)LinCharger_OnMasterFrame(s_rx_pid, s_master_buf, s_master_len, rsp, &rsp_len);

  if ((rsp_len > 0U) && (frame->slave_len > 0U))
  {
    uint8_t tx_len = rsp_len;

    if (tx_len > frame->slave_len)
    {
      tx_len = frame->slave_len;
    }

    (void)memcpy(s_tx_buf, rsp, tx_len);
    s_tx_buf[tx_len] = lin_checksum_enhanced(s_rx_pid, s_tx_buf, tx_len);
    (void)lin_send_slave_frame(tx_len);
    return;
  }

  s_state = LIN_DRV_IDLE;
}

static void lin_start_slave_tx(void)
{
  uint8_t rsp[8];
  uint8_t rsp_len = 0U;

  (void)LinCharger_OnMasterFrame(s_rx_pid, NULL, 0U, rsp, &rsp_len);

  if (rsp_len > s_slave_len)
  {
    rsp_len = s_slave_len;
  }

  if (rsp_len == 0U)
  {
    s_state = LIN_DRV_IDLE;
    return;
  }

  (void)memcpy(s_tx_buf, rsp, rsp_len);
  s_tx_buf[rsp_len] = lin_checksum_enhanced(s_rx_pid, s_tx_buf, rsp_len);
  (void)lin_send_slave_frame(rsp_len);
}

static void lin_on_break(void)
{
  (void)HAL_UART_AbortReceive_IT(&huart1);
  (void)HAL_UART_AbortTransmit_IT(&huart1);
  lin_clear_uart_errors();
  lin_flush_rx();

  s_diag.break_count++;
  s_state = LIN_DRV_RX_SYNC;
  lin_arm_rx_byte();
}

void LinDriver_Init(void)
{
  if (s_inited)
  {
    return;
  }

  if (huart1.gState != HAL_UART_STATE_RESET)
  {
    (void)HAL_UART_DeInit(&huart1);
  }

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
#if BMS_LIN_USART_SWAP_ENABLE
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
#else
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif

  if (HAL_LIN_Init(&huart1, UART_LINBREAKDETECTLENGTH_11B) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_LBD);

  s_state = LIN_DRV_IDLE;
  s_inited = true;
}

void LinDriver_IRQHandler(void)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_LBDF))
  {
    if (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_LBD))
    {
      __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_LBDF);
      lin_on_break();
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  const lin_frame_map_t *frame;

  if (huart->Instance != USART1)
  {
    return;
  }

  switch (s_state)
  {
    case LIN_DRV_RX_SYNC:
      if (s_rx_byte == 0x55U)
      {
        s_diag.sync_ok++;
        s_state = LIN_DRV_RX_PID;
        lin_arm_rx_byte();
      }
      else
      {
        s_diag.sync_fail++;
        s_state = LIN_DRV_IDLE;
      }
      break;

    case LIN_DRV_RX_PID:
      s_rx_pid = s_rx_byte;
      frame = lin_find_frame((uint8_t)(s_rx_pid & 0x3Fu));
      if (frame == NULL)
      {
        s_diag.pid_miss++;
        s_state = LIN_DRV_IDLE;
        break;
      }

      s_master_len = frame->master_len;
      s_slave_len = frame->slave_len;
      s_master_idx = 0U;
      s_master_rx_total = (uint8_t)(s_master_len + ((s_master_len > 0U) ? 1U : 0U));

      if (s_master_rx_total > 0U)
      {
        s_state = LIN_DRV_RX_MASTER;
        lin_arm_rx_master();
      }
      else if (s_slave_len > 0U)
      {
        lin_start_slave_tx();
      }
      else
      {
        s_state = LIN_DRV_IDLE;
      }
      break;

    case LIN_DRV_RX_MASTER:
      s_master_idx++;
      if (s_master_idx < s_master_rx_total)
      {
        lin_arm_rx_master();
      }
      else
      {
        lin_finish_master_rx();
      }
      break;

    default:
      s_state = LIN_DRV_IDLE;
      break;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  if (s_state == LIN_DRV_TX_SLAVE)
  {
    s_state = LIN_DRV_IDLE;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  lin_clear_uart_errors();
  s_state = LIN_DRV_IDLE;
}

const lin_driver_diag_t *LinDriver_GetDiag(void)
{
  return &s_diag;
}
