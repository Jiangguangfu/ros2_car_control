/**
 * @file    lin_diag_tx.c
 * @brief   BMS LIN 硬件排查：USART1 @ 19200 周期发测试帧，供 LA 在座子上验证。
 */
#include "lin_diag_tx.h"

#include "bms_lin_config.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

#define LIN_DIAG_BREAK_HOLD_MS    1U
#define LIN_DIAG_IO_TIMEOUT_MS    50U

static uint32_t s_tx_frames;
static uint8_t s_inited;

static void lin_diag_wait_break_sent(void)
{
  uint32_t start = HAL_GetTick();

  while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC))
  {
    if ((uint32_t)(HAL_GetTick() - start) >= 15U)
    {
      break;
    }
  }

  HAL_Delay(LIN_DIAG_BREAK_HOLD_MS);
}

void LinDiagTx_Init(void)
{
  if (s_inited != 0U)
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

  s_tx_frames = 0U;
  s_inited = 1U;
}

void LinDiagTx_Poll(void)
{
  static const uint8_t header[] = {0x55U, 0x10U};
  static const uint8_t payload[] = {0x01U, 0x01U};

  if (s_inited == 0U)
  {
    LinDiagTx_Init();
  }

  if (HAL_LIN_SendBreak(&huart1) != HAL_OK)
  {
    return;
  }

  lin_diag_wait_break_sent();

  if (HAL_UART_Transmit(&huart1, (uint8_t *)header, sizeof(header),
                        LIN_DIAG_IO_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }

  if (HAL_UART_Transmit(&huart1, (uint8_t *)payload, sizeof(payload),
                        LIN_DIAG_IO_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }

  s_tx_frames++;
}

uint32_t LinDiagTx_GetFrameCount(void)
{
  return s_tx_frames;
}
