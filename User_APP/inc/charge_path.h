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
#include <stdint.h>
#include "charge_reject.h"

#ifdef __cplusplus
extern "C" {
#endif

void ChargePath_Init(void);

/** Thermal manager request (charge and/or discharge inhibit). */
void ChargePath_SetThermalInhibit(bool charge_off, bool discharge_off);

/**
 * Stop charging only (top fine-balance or mid highest-cell protect may run).
 * Mid: relax classify / vmax protect. Top: large Δ pause (see balance manager).
 */
void ChargePath_SetImbalanceChargeInhibit(bool charge_off);

/** Charge manager: inhibit when idle/stop/complete/fault; allow when CHARGING. */
void ChargePath_SetChargeManagerInhibit(bool charge_off);

/** Overcurrent / short-circuit protect manager. */
void ChargePath_SetProtectInhibit(bool charge_off, bool discharge_off);

/** Cell voltage protect: COV stop charge, CUV stop discharge. */
void ChargePath_SetVoltageInhibit(bool charge_off, bool discharge_off);

/** LIN 通信超时：暂停充电路径（不断开 CHG 状态机）。 */
void ChargePath_SetLinCommInhibit(bool charge_off);

/**
 * Boot / discharge path gate: default inhibit until self-check releases (S1/S2).
 * discharge_off=true → DFETOFF high (block DSG/PDSG).
 */
void ChargePath_SetBootDischargeInhibit(bool discharge_off);

/** Drive CFETOFF/DFETOFF from OR of all requests. */
void ChargePath_Apply(void);

bool ChargePath_IsImbalanceChargeInhibit(void);
bool ChargePath_IsLinCommInhibit(void);
bool ChargePath_IsChargeInhibited(void);
bool ChargePath_IsDischargeInhibited(void);

/** 当前充电路径禁止来源（不含 charge_manager 空闲关断） */
uint16_t ChargePath_GetChargeInhibitMask(void);

#ifdef __cplusplus
}
#endif

#endif /* CHARGE_PATH_H */
