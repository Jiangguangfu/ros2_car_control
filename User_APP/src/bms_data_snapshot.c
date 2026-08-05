/**
 * @file    bms_data_snapshot.c
 * @brief   BQ76942 测量值 → uart_battery_state_report_t
 */
#include "bms_data_snapshot.h"

#include "app_freertos.h"
#include "bq76942.h"
#include "soc_estimator.h"
#include "thermal_manager.h"

static float bms_snapshot_temp_c(const bq76942_temp_t *temp)
{
  bool ts1_ok;
  bool ts2_ok;
  int16_t tmax_c_x10;

  if (temp == NULL || !temp->valid) {
    return -1.0f;
  }

  ts1_ok = (temp->ts1_temp_c_x10 > -400) && (temp->ts1_temp_c_x10 < 1250);
  ts2_ok = (temp->ts2_temp_c_x10 > -400) && (temp->ts2_temp_c_x10 < 1250);

  if (ts1_ok && ts2_ok) {
    tmax_c_x10 = (temp->ts1_temp_c_x10 > temp->ts2_temp_c_x10) ?
                 temp->ts1_temp_c_x10 : temp->ts2_temp_c_x10;
    return (float)tmax_c_x10 / 10.0f;
  }
  if (ts1_ok) {
    return (float)temp->ts1_temp_c_x10 / 10.0f;
  }
  if (ts2_ok) {
    return (float)temp->ts2_temp_c_x10 / 10.0f;
  }

  return -1.0f;
}

void BmsDataSnapshot_Fill(uart_battery_state_report_t *out)
{
  const bq76942_meas_t *meas;
  const bq76942_temp_t *temp;
  const thermal_status_t *thermal;

  if (out == NULL) {
    return;
  }

  meas = Bms_GetBqMeasurements();
  temp = Bms_GetBqTemperatures();
  thermal = Thermal_GetStatus();

  out->series_cells = (uint8_t)BQ76942_CELL_COUNT;
  out->present = 1u;
  out->reserved0 = 0u;
  out->reserved1 = 0u;
  out->percentage = -1.0f;

  if (Soc_IsValid()) {
    out->percentage = (float)Soc_GetPercent() / 100.0f;
  }

  if (meas != NULL && meas->valid && meas->pack_mv > 0U) {
    out->voltage_v = (float)meas->pack_mv / 1000.0f;
    /* BQ: + charge / - discharge；协议：放电为正 */
    out->current_a = -(float)meas->current_ma / 1000.0f;
    out->reserved1 = BMS_BATTERY_REPORT_VALID_BIT;
  } else {
    out->voltage_v = 0.0f;
    out->current_a = 0.0f;
  }

  out->temperature_c = bms_snapshot_temp_c(temp);
  if ((thermal != NULL) && !thermal->sensor_ok && (out->temperature_c < 0.0f)) {
    out->temperature_c = -1.0f;
  }
}
