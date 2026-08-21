/**
 * @file    charge_gate.h
 * @brief   开充前只读安全仲裁（与 charge_path / ChargeManager 预检同一条件）
 */
#ifndef CHARGE_GATE_H
#define CHARGE_GATE_H

#include <stdbool.h>
#include "charge_reject.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @param require_charger true：ROS 开充，需 LIN 已 V/I 协商且未丢帧
 */
void ChargeGate_Evaluate(bool require_charger, charge_gate_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CHARGE_GATE_H */
