/**
 * @file    bms_can_debug.c
 * @brief   BQ 原始采样经 CAN 输出（PCAN 抓取，无需串口）
 *
 * 与 0x48B 同周期发送：
 *   0x48C  [flags][mf][tf][pack_L][pack_H][I_L][I_H][eff/100]
 *   0x48D  [out_L][out_H][c1_L][c1_H][c2_L][c2_H][c3_L][c3_H]
 *   0x48E  [c4_L][c4_H][c5_L][c5_H][c6_L][c6_H][ts1_L][ts1_H]
 *   0x48F  [ts2_L][ts2_H][cc3_L][cc3_H][00][00][00][00]
 *
 * flags: bit0 meas.valid  bit1 temp.valid  bit2 CAN 0x48B valid  bit3 BQ76942_IsReady
 * 电压单位 mV (u16 LE)；温度 0.1°C (s16 LE)；电流 mA (s16 LE, BQ 符号)
 */
#include "bms_can_debug.h"

#if (BMS_CAN_DEBUG_ENABLE != 0)

#include "app_freertos.h"
#include "bms_data_snapshot.h"
#include "bq76942.h"
#include "main.h"
#include "uart_battery_report.h"

#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c2;

#define BMS_CAN_ID_DEBUG_A  0x48Cu
#define BMS_CAN_ID_DEBUG_B  0x48Du
#define BMS_CAN_ID_DEBUG_C  0x48Eu
#define BMS_CAN_ID_DEBUG_D  0x48Fu
#define BMS_CAN_TX_FIFO_WAIT_MS  10U

static bool bms_can_debug_wait_fifo(void)
{
  uint32_t t0 = HAL_GetTick();

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 1U) {
    if ((HAL_GetTick() - t0) >= BMS_CAN_TX_FIFO_WAIT_MS) {
      return false;
    }
  }

  return true;
}

static bool bms_can_debug_send(uint16_t can_id, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef hdr;

  if ((hfdcan1.Instance == NULL) || !bms_can_debug_wait_fifo()) {
    return false;
  }

  hdr.Identifier = can_id;
  hdr.IdType = FDCAN_STANDARD_ID;
  hdr.TxFrameType = FDCAN_DATA_FRAME;
  hdr.DataLength = FDCAN_DLC_BYTES_8;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch = FDCAN_BRS_OFF;
  hdr.FDFormat = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker = 0;

  return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, (uint8_t *)data) == HAL_OK);
}

static void bms_can_debug_put_u16(uint8_t *buf, uint8_t off, uint16_t val)
{
  buf[off] = (uint8_t)(val & 0xFFU);
  buf[off + 1U] = (uint8_t)((val >> 8) & 0xFFU);
}

static void bms_can_debug_put_s16(uint8_t *buf, uint8_t off, int16_t val)
{
  bms_can_debug_put_u16(buf, off, (uint16_t)val);
}

void BMS_CanDebug_Init(void)
{
}

void BMS_CanDebug_Process(void)
{
  const bq76942_meas_t *meas = Bms_GetBqMeasurements();
  const bq76942_temp_t *temp = Bms_GetBqTemperatures();
  uart_battery_state_report_t report;
  uint8_t f0[8];
  uint8_t f1[8];
  uint8_t f2[8];
  uint8_t f3[8];
  uint32_t eff_mv;
  uint8_t flags = 0U;

  if (hfdcan1.Instance == NULL) {
    return;
  }

  BmsDataSnapshot_Fill(&report);
  eff_mv = BmsDataSnapshot_PackMv(meas);

  if ((meas != NULL) && meas->valid) {
    flags = (uint8_t)(flags | 0x01U);
  }
  if ((temp != NULL) && temp->valid) {
    flags = (uint8_t)(flags | 0x02U);
  }
  if ((report.reserved1 & BMS_BATTERY_REPORT_VALID_BIT) != 0U) {
    flags = (uint8_t)(flags | 0x04U);
  }
  if (BQ76942_IsReady(&hi2c2)) {
    flags = (uint8_t)(flags | 0x08U);
  }

  (void)memset(f0, 0, sizeof(f0));
  f0[0] = flags;
  f0[1] = (uint8_t)((Bms_GetBqMeasFailCount() > 255U) ? 255U : Bms_GetBqMeasFailCount());
  f0[2] = (uint8_t)((Bms_GetBqTempFailCount() > 255U) ? 255U : Bms_GetBqTempFailCount());
  if (meas != NULL) {
    bms_can_debug_put_u16(f0, 3U, (uint16_t)((meas->pack_mv > 0xFFFFU) ? 0xFFFFU : meas->pack_mv));
    bms_can_debug_put_s16(f0, 5U, meas->current_ma);
  }
  f0[7] = (uint8_t)((eff_mv > 25500U) ? 255U : (eff_mv / 100U));

  (void)memset(f1, 0, sizeof(f1));
  if (meas != NULL) {
    bms_can_debug_put_u16(f1, 0U, (uint16_t)((meas->output_mv > 0xFFFFU) ? 0xFFFFU : meas->output_mv));
    bms_can_debug_put_u16(f1, 2U, meas->cell_mv[0]);
    bms_can_debug_put_u16(f1, 4U, meas->cell_mv[1]);
    bms_can_debug_put_u16(f1, 6U, meas->cell_mv[2]);
  }

  (void)memset(f2, 0, sizeof(f2));
  if (meas != NULL) {
    bms_can_debug_put_u16(f2, 0U, meas->cell_mv[3]);
    bms_can_debug_put_u16(f2, 2U, meas->cell_mv[4]);
    bms_can_debug_put_u16(f2, 4U, meas->cell_mv[5]);
  }
  if (temp != NULL) {
    bms_can_debug_put_s16(f2, 6U, temp->ts1_temp_c_x10);
  }

  (void)memset(f3, 0, sizeof(f3));
  if (temp != NULL) {
    bms_can_debug_put_s16(f3, 0U, temp->ts2_temp_c_x10);
  }
  if (meas != NULL) {
    bms_can_debug_put_s16(f3, 2U, meas->current_cc3_ma);
  }

  (void)bms_can_debug_send(BMS_CAN_ID_DEBUG_A, f0);
  (void)bms_can_debug_send(BMS_CAN_ID_DEBUG_B, f1);
  (void)bms_can_debug_send(BMS_CAN_ID_DEBUG_C, f2);
  (void)bms_can_debug_send(BMS_CAN_ID_DEBUG_D, f3);
}

#else

void BMS_CanDebug_Init(void)
{
}

void BMS_CanDebug_Process(void)
{
}

#endif /* BMS_CAN_DEBUG_ENABLE */
