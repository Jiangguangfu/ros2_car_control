/**
 * @file    bms_ext_snapshot.c
 * @brief   告警与扩展测量 → uart_battery_ext_report_t（0x9A / CAN 0x49A）
 */
#include "bms_ext_snapshot.h"

#include "app_freertos.h"
#include "bq76942.h"
#include "bsp_power_rails.h"
#include "cell_balance_manager.h"
#include "charge_manager.h"
#include "cell_voltage_protect.h"

#include <string.h>

#define BMS_EXT_COMM_FAIL_THRESHOLD  3U

static int16_t bms_ext_clamp_ts1_x10(int16_t t_x10)
{
  if ((t_x10 > -400) && (t_x10 < 1250)) {
    return t_x10;
  }

  return (int16_t)-4000;
}

void BmsExtSnapshot_Fill(uart_battery_ext_report_t *out)
{
  const bq76942_meas_t *meas;
  const bq76942_temp_t *temp;
  const pwr_rails_status_t *pwr;
  const charge_status_t *charge;
  const balance_status_t *balance;
  uint32_t alarm = 0U;
  uint8_t severity = BMS_EXT_SEVERITY_NONE;
  uint8_t source = 0U;
  uint8_t i;

  if (out == NULL) {
    return;
  }

  meas = Bms_GetBqMeasurements();
  temp = Bms_GetBqTemperatures();
  pwr = BSP_PowerRails_GetStatus();
  charge = ChargeManager_GetStatus();
  balance = Balance_GetStatus();

  (void)memset(out, 0, sizeof(*out));

  if ((meas != NULL) && meas->valid) {
    source = (uint8_t)(source | BMS_EXT_SOURCE_BQ_MEAS);
    out->output_mv = (uint16_t)((meas->output_mv > 0xFFFFU) ? 0xFFFFU : meas->output_mv);
    out->vcell_min_mv = meas->vcell_min_mv;
    out->vcell_max_mv = meas->vcell_max_mv;
    out->current_cc2_ma = meas->current_ma;
    out->current_cc3_ma = meas->current_cc3_ma;

    for (i = 0U; i < BQ76942_CELL_COUNT; i++) {
      out->cell_mv[i] = meas->cell_mv[i];
    }

    if (meas->vcell_max_mv >= BALANCE_NORMAL_MAX_CELL_MV) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_OVP);
    }
    if (meas->vcell_min_mv > 0U && meas->vcell_min_mv <= BALANCE_HARD_MIN_CELL_MV) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_UVP);
    }
  }

  if (CellVoltageProtect_IsValid()) {
    source = (uint8_t)(source | BMS_EXT_SOURCE_PROTECT);

    if (CellVoltageProtect_IsCov()) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_OVP | BMS_EXT_ALARM_BQ_PROTECT);
    }
    if (CellVoltageProtect_IsCuv()) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_UVP | BMS_EXT_ALARM_BQ_PROTECT);
    }
    if (CellVoltageProtect_IsLowVoltageWarn()) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_LOW_BATTERY);
    }
  }

  if ((temp != NULL) && temp->valid) {
    source = (uint8_t)(source | BMS_EXT_SOURCE_BQ_MEAS);
    out->ts1_c_x10 = bms_ext_clamp_ts1_x10(temp->ts1_temp_c_x10);
    out->ts2_c_x10 = bms_ext_clamp_ts1_x10(temp->ts2_temp_c_x10);
  } else {
    out->ts1_c_x10 = (int16_t)-4000;
    out->ts2_c_x10 = (int16_t)-4000;
  }

  if (pwr != NULL) {
    source = (uint8_t)(source | BMS_EXT_SOURCE_THERMAL | BMS_EXT_SOURCE_PROTECT);

    if ((pwr->state >= PWR_STATE_LIMIT) &&
        ((pwr->reason == PWR_REASON_HOT) ||
         (pwr->reason == PWR_REASON_SENSOR) ||
         (pwr->reason == PWR_REASON_COLD_CHARGE))) {
      if (pwr->reason == PWR_REASON_COLD_CHARGE) {
        alarm = (uint32_t)(alarm | BMS_EXT_ALARM_COLD_CHARGE);
      } else {
        alarm = (uint32_t)(alarm | BMS_EXT_ALARM_OVERTEMP);
      }
    }

    if (pwr->reason == PWR_REASON_SCD) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_SHORT_CIRCUIT |
                         BMS_EXT_ALARM_OCP | BMS_EXT_ALARM_BQ_PROTECT);
    } else if ((pwr->reason == PWR_REASON_OCD) ||
               (pwr->reason == PWR_REASON_OCC) ||
               (pwr->reason == PWR_REASON_SOFT_OCD) ||
               (pwr->state == PWR_STATE_WARN &&
                pwr->reason == PWR_REASON_SOFT_OCD)) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_OCP);
      if (pwr->reason != PWR_REASON_SOFT_OCD) {
        alarm = (uint32_t)(alarm | BMS_EXT_ALARM_BQ_PROTECT);
      }
    }

    if (pwr->charge_inhibit) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_CHG_INHIBIT);
    }
    if (pwr->discharge_inhibit) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_DSG_INHIBIT);
    }
    if (!pwr->sensor_ok) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_COMM_FAIL);
    }
    if (pwr->bq_valid && pwr->bq_any) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_BQ_PROTECT);
    }
  }

  if (charge != NULL) {
    source = (uint8_t)(source | BMS_EXT_SOURCE_CHARGE);

    if (charge->state == CHARGE_STATE_FAULT) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_CHARGE_FAULT);

      if (charge->fault_reason == CHARGE_FAULT_OVERCURRENT) {
        alarm = (uint32_t)(alarm | BMS_EXT_ALARM_OCP);
      }
      if (charge->fault_reason == CHARGE_FAULT_OVERVOLT) {
        alarm = (uint32_t)(alarm | BMS_EXT_ALARM_OVP);
      }
      if (charge->fault_reason == CHARGE_FAULT_UNDERVOLT) {
        alarm = (uint32_t)(alarm | BMS_EXT_ALARM_UVP);
      }
      if (charge->fault_reason == CHARGE_FAULT_BQ_PROTECT) {
        alarm = (uint32_t)(alarm | BMS_EXT_ALARM_BQ_PROTECT);
      }
      if (charge->fault_reason == CHARGE_FAULT_COMM) {
        alarm = (uint32_t)(alarm | BMS_EXT_ALARM_COMM_FAIL);
      }
      if (charge->fault_reason == CHARGE_FAULT_IMBALANCE) {
        alarm = (uint32_t)(alarm | BMS_EXT_ALARM_IMBALANCE_CHG);
      }
    }
  }

  if (balance != NULL) {
    source = (uint8_t)(source | BMS_EXT_SOURCE_BALANCE);

    if (balance->imbalance_charge_inhibit) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_IMBALANCE_CHG);
    }
    if ((balance->state == BALANCE_STATE_ACTIVE) ||
        (balance->state == BALANCE_STATE_MID_PROTECT)) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_BALANCING);
    }
    if (!balance->delta_ok &&
        (balance->top_start_ready ||
         (balance->state == BALANCE_STATE_ACTIVE))) {
      alarm = (uint32_t)(alarm | BMS_EXT_ALARM_DELTA_HIGH);
    }
  }

  if (Bms_GetBqMeasFailCount() >= BMS_EXT_COMM_FAIL_THRESHOLD ||
      Bms_GetBqTempFailCount() >= BMS_EXT_COMM_FAIL_THRESHOLD ||
      Bms_GetBqCommFailCount() >= BMS_EXT_COMM_FAIL_THRESHOLD) {
    alarm = (uint32_t)(alarm | BMS_EXT_ALARM_COMM_FAIL);
  }

  {
    uint32_t sev_alarms = (uint32_t)(alarm & ~BMS_EXT_ALARM_BALANCING);

    if (sev_alarms != 0U) {
      if ((sev_alarms & (BMS_EXT_ALARM_BQ_PROTECT |
                         BMS_EXT_ALARM_CHARGE_FAULT |
                         BMS_EXT_ALARM_OVERTEMP |
                         BMS_EXT_ALARM_SHORT_CIRCUIT |
                         BMS_EXT_ALARM_OCP)) != 0U) {
        severity = BMS_EXT_SEVERITY_CRITICAL;
      } else {
        severity = BMS_EXT_SEVERITY_WARN;
      }
    }
  }

  out->alarm_flags = alarm;
  out->severity = severity;
  out->source_flags = source;
}
