/**
 * @brief  FDCAN smoke — PA11/PA12, 500k, ID 0x123.
 *         CommTask 每 200ms 入队；AutoRetransmission OFF；Bus-Off 时 Stop/Start。
 */
#include "bms_can_bench.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

static void BMS_Can_RecoverIfBusOff(void)
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

void BMS_CanBench_Init(void)
{
}

void BMS_CanBench_Process(void)
{
  static const uint8_t smoke[8] = {0xAAU, 0x55U, 0x12U, 0x34U, 0x56U, 0x78U, 0x9AU, 0xBCU};
  FDCAN_TxHeaderTypeDef hdr;

  if (hfdcan1.Instance == NULL) {
    return;
  }

  BMS_Can_RecoverIfBusOff();

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 1U) {
    return;
  }

  hdr.Identifier = 0x123U;
  hdr.IdType = FDCAN_STANDARD_ID;
  hdr.TxFrameType = FDCAN_DATA_FRAME;
  hdr.DataLength = FDCAN_DLC_BYTES_8;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch = FDCAN_BRS_OFF;
  hdr.FDFormat = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker = 0;

  (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, (uint8_t *)smoke);
}
