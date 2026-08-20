/**
 * @file    uart_charge_ctrl.h
 * @brief   407 → BMS 充电控制（UART TYPE 0x41 / CAN 0x441）
 */
#ifndef UART_CHARGE_CTRL_H
#define UART_CHARGE_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define UART_TYPE_SET_CHARGE_CTRL       0x41u

#define UART_CHARGE_CTRL_SET_CURRENT    0u
#define UART_CHARGE_CTRL_START          1u
#define UART_CHARGE_CTRL_STOP           2u

/* Keep in sync with the 407 and charger BQ25756 limits. */
#define UART_CHARGE_CURRENT_MIN_MA       400u
#define UART_CHARGE_CURRENT_MAX_MA       3000u
#define UART_CHARGE_CURRENT_STEP_MA       50u

static inline int uart_charge_current_is_valid(uint16_t current_ma)
{
  return (current_ma >= UART_CHARGE_CURRENT_MIN_MA) &&
         (current_ma <= UART_CHARGE_CURRENT_MAX_MA) &&
         ((current_ma % UART_CHARGE_CURRENT_STEP_MA) == 0u);
}

typedef struct __attribute__((packed)) {
  uint8_t cmd;
  uint16_t i_target_ma;
} uart_charge_ctrl_cmd_t;

#ifdef __cplusplus
}
#endif

#endif /* UART_CHARGE_CTRL_H */
