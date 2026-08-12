/**
 * @file    lin_diag_tx.h
 * @brief   BMS LIN 硬件排查：仅 Master 式发 Break+Sync+测试数据
 */
#ifndef LIN_DIAG_TX_H
#define LIN_DIAG_TX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void LinDiagTx_Init(void);
void LinDiagTx_Poll(void);
uint32_t LinDiagTx_GetFrameCount(void);

#ifdef __cplusplus
}
#endif

#endif /* LIN_DIAG_TX_H */
