/**
 * @file    lin_driver.h
 * @brief   USART1 LIN 从机驱动（19200，接板载 LIN 座子）
 */
#ifndef LIN_DRIVER_H
#define LIN_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void LinDriver_Init(void);
void LinDriver_IRQHandler(void);

typedef struct
{
  uint32_t break_count;
  uint32_t sync_ok;
  uint32_t sync_fail;
  uint32_t cs_fail;
  uint32_t pid_miss;
  uint32_t rsp_tx;
} lin_driver_diag_t;

const lin_driver_diag_t *LinDriver_GetDiag(void);

#ifdef __cplusplus
}
#endif

#endif /* LIN_DRIVER_H */
