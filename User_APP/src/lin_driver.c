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
#include "stm32u3xx_hal_uart_ex.h"

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
    {0x32u, 0u, 8u}, /* 充电状态轮询 */
    {0x33u, 0u, 8u}, /* 均衡监控轮询 */
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
static bool s_after_sync;
static uint8_t s_tx_buf[9];
static volatile bool s_inited;

static lin_driver_diag_t s_diag;

#define LIN_INIT_FLAG_MAIN      0x01U
#define LIN_INIT_FLAG_SWAP      0x02U

static void lin_go_idle(void);
static void lin_idle_rx_enable(void);
static void lin_idle_rx_disable(void);
static void lin_arm_rx_master(void);
static void lin_start_slave_tx(void);
static void lin_begin_master_frame(const lin_frame_map_t *frame, uint8_t pid);
static void lin_idle_rx_byte(uint8_t byte);
static void lin_service_idle_rx(void);
static void lin_on_break(void);

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

static void lin_drain_rx(void)
{
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
  {
    const uint8_t byte = (uint8_t)(READ_REG(huart1.Instance->RDR) & 0xFFU);

    if (s_state == LIN_DRV_IDLE)
    {
      lin_idle_rx_byte(byte);
    }
  }
}

static void lin_clear_uart_errors(void)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
  {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
      (void)READ_REG(huart1.Instance->RDR);
    }
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
    lin_go_idle();
    return false;
  }

  lin_go_idle();
  return true;
}

static void lin_arm_rx_byte(void)
{
  lin_idle_rx_disable();
  if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK)
  {
    lin_go_idle();
  }
}

static void lin_idle_rx_enable(void)
{
  if (s_state == LIN_DRV_IDLE)
  {
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  }
}

static void lin_idle_rx_disable(void)
{
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
}

static void lin_go_idle(void)
{
  s_after_sync = false;
  s_state = LIN_DRV_IDLE;
  lin_idle_rx_enable();
}

static void lin_begin_master_frame(const lin_frame_map_t *frame, uint8_t pid)
{
  s_rx_pid = pid;
  s_master_len = frame->master_len;
  s_slave_len = frame->slave_len;
  s_master_idx = 0U;
  s_master_rx_total = (uint8_t)(s_master_len + ((s_master_len > 0U) ? 1U : 0U));
  s_after_sync = false;

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
    lin_go_idle();
  }
}

static void lin_sniff_record(uint8_t byte)
{
  const uint32_t pos = s_diag.sniff_pos & 0x0FU;
  const uint32_t word = pos >> 2U;
  const uint32_t shift = (pos & 3U) * 8U;

  s_diag.sniff_w[word] &= ~(0xFFUL << shift);
  s_diag.sniff_w[word] |= ((uint32_t)byte << shift);
  s_diag.sniff_pos = (pos + 1U) & 0x0FU;
}

static void lin_idle_rx_byte(uint8_t byte)
{
  const lin_frame_map_t *frame;

  s_diag.rx_sniff++;
  lin_sniff_record(byte);

  if (s_after_sync)
  {
    s_after_sync = false;
    frame = lin_find_frame((uint8_t)(byte & 0x3Fu));
    if (frame != NULL)
    {
      s_diag.sync_sniff++;
      s_diag.sync_ok++;
      lin_begin_master_frame(frame, byte);
      return;
    }

    s_diag.last_miss_pid = byte;
    s_diag.pid_miss++;
  }

  s_after_sync = (byte == 0x55U);
}

/* 空闲监听维护：取走 RX 字节，并清掉不会被读 RDR 清除的 ORE/NE。 */
static void lin_service_idle_rx(void)
{
  lin_drain_rx();

  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    s_diag.uart_err++;
  }
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE))
  {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_NEF);
  }
  /* FE 只能靠 ICR 清；留着会让 HAL 永远走不进无错分支。 */
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE))
  {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
    s_diag.fe_count++;
    lin_on_break();
  }
}

static void lin_arm_rx_master(void)
{
  lin_idle_rx_disable();
  if (HAL_UART_Receive_IT(&huart1, &s_master_buf[s_master_idx], 1U) != HAL_OK)
  {
    lin_go_idle();
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
    lin_go_idle();
    return;
  }

  if (s_master_len > 0U)
  {
    cs = lin_checksum_enhanced(s_rx_pid, s_master_buf, s_master_len);
    if (s_master_buf[s_master_len] != cs)
    {
      s_diag.cs_fail++;
      lin_go_idle();
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

  lin_go_idle();
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
    lin_go_idle();
    return;
  }

  (void)memcpy(s_tx_buf, rsp, rsp_len);
  s_tx_buf[rsp_len] = lin_checksum_enhanced(s_rx_pid, s_tx_buf, rsp_len);
  (void)lin_send_slave_frame(rsp_len);
}

static void lin_on_break(void)
{
  lin_idle_rx_disable();
  (void)HAL_UART_AbortReceive_IT(&huart1);
  (void)HAL_UART_AbortTransmit_IT(&huart1);
  s_after_sync = false;
  lin_clear_uart_errors();
  lin_flush_rx();

  s_diag.break_count++;
  s_state = LIN_DRV_RX_SYNC;
  lin_arm_rx_byte();
}

static void lin_handle_lbd(void)
{
  if (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_LBDF))
  {
    return;
  }

  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_LBDF);
  lin_on_break();
}

static void lin_driver_apply_init(uint32_t init_flags)
{
  if (huart1.gState != HAL_UART_STATE_RESET)
  {
    (void)HAL_UART_DeInit(&huart1);
  }

  (void)memset(&huart1, 0, sizeof(huart1));

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

  (void)HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_DisableFifoMode(&huart1);
  lin_flush_rx();

  SET_BIT(huart1.Instance->CR2, USART_CR2_LBDIE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_LBD);

  s_after_sync = false;
  s_state = LIN_DRV_IDLE;
  s_diag.init_flags = init_flags;
#if BMS_LIN_USART_SWAP_ENABLE
  s_diag.init_flags |= LIN_INIT_FLAG_SWAP;
#endif
  lin_idle_rx_enable();
}

void LinDriver_Init(void)
{
  if (s_inited)
  {
    return;
  }

  lin_driver_apply_init(LIN_INIT_FLAG_MAIN);
  s_inited = true;
}

void LinDriver_IRQHandler(void)
{
  lin_handle_lbd();

  if (s_state == LIN_DRV_IDLE)
  {
    lin_service_idle_rx();
  }
}

void LinDriver_Poll(void)
{
  if (!s_inited)
  {
    return;
  }

  lin_handle_lbd();

  if (s_state == LIN_DRV_IDLE)
  {
    lin_service_idle_rx();
    lin_idle_rx_enable();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
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
        lin_go_idle();
      }
      break;

    case LIN_DRV_RX_PID:
    {
      const lin_frame_map_t *frame = lin_find_frame((uint8_t)(s_rx_byte & 0x3Fu));

      if (frame == NULL)
      {
        s_diag.last_miss_pid = s_rx_byte;
        s_diag.pid_miss++;
        lin_go_idle();
        break;
      }

      lin_begin_master_frame(frame, s_rx_byte);
      break;
    }

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
      lin_go_idle();
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
    lin_go_idle();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t err;

  if (huart->Instance != USART1)
  {
    return;
  }

  err = huart->ErrorCode;
  s_diag.last_err = err;
  s_diag.uart_err++;

  lin_drain_rx();

  if (((err & HAL_UART_ERROR_FE) != 0U) && (s_state == LIN_DRV_IDLE))
  {
    s_diag.fe_break++;
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_FEF);
    lin_on_break();
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    return;
  }

  (void)HAL_UART_AbortReceive_IT(&huart1);
  (void)HAL_UART_AbortTransmit_IT(&huart1);
  lin_clear_uart_errors();
  lin_go_idle();
  huart->ErrorCode = HAL_UART_ERROR_NONE;
}

const lin_driver_diag_t *LinDriver_GetDiag(void)
{
  return &s_diag;
}
