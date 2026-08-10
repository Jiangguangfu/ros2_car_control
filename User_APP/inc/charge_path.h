/**
 ******************************************************************************
 * @file    charge_path.h
 * @brief   Charge/discharge FET-off arbitration (OR of request sources).
 *
 * CFETOFF/DFETOFF are active-high inhibit on this board. Multiple modules
 * request inhibit; ChargePath_Apply() drives the pins.
 ******************************************************************************
 */
#ifndef CHARGE_PATH_H
#define CHARGE_PATH_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void ChargePath_Init(void);

/** Thermal manager request (charge and/or discharge inhibit). */
void ChargePath_SetThermalInhibit(bool charge_off, bool discharge_off);

/**
 * Cell imbalance request: stop charging only (passive balance may continue).
 * Assert when Δ ≥ 50 mV; clear when Δ ≤ 30 mV.
 */
void ChargePath_SetImbalanceChargeInhibit(bool charge_off);

/** Charge manager: inhibit when idle/stop/complete/fault; allow when CHARGING. */
void ChargePath_SetChargeManagerInhibit(bool charge_off);

/** Overcurrent / short-circuit protect manager. */
void ChargePath_SetProtectInhibit(bool charge_off, bool discharge_off);

/** Drive CFETOFF/DFETOFF from OR of all requests. */
void ChargePath_Apply(void);

bool ChargePath_IsImbalanceChargeInhibit(void);
bool ChargePath_IsChargeInhibited(void);
bool ChargePath_IsDischargeInhibited(void);

#ifdef __cplusplus
}
#endif

#endif /* CHARGE_PATH_H */
