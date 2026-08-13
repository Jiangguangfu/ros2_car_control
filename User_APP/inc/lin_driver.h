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
void LinDriver_Poll(void);

typedef struct
{
  uint32_t break_count;
  uint32_t sync_ok;
  uint32_t sync_fail;
  uint32_t cs_fail;
  uint32_t pid_miss;
  uint32_t rsp_tx;
  uint32_t init_flags;   /* bit0=main 早Init, bit1=Swap开启 */
  uint32_t fe_break;     /* FE 兜底当 Break 次数（LBD 未触发时） */
  uint32_t rx_sniff;     /* 空闲时 RX 收到任意字节 */
  uint32_t uart_err;     /* USART 错误（FE/ORE 等） */
  uint32_t sync_sniff;   /* 0x55+有效PID 兜底入帧 */
  uint32_t last_miss_pid; /* 最近一次 0x55 后无效 PID 字节 */
  uint32_t last_err;     /* 最近一次 HAL UART ErrorCode */
  uint32_t fe_count;     /* 收到 FE 次数（Break 若未被 LBD 认出会记在这） */
  uint32_t sniff_w[4];   /* 最近 16 个空闲字节，小端打包，配合 sniff_pos 判断顺序 */
  uint32_t sniff_pos;    /* 下一个写入位置（0..15），累计写入数 = rx_sniff */
} lin_driver_diag_t;

const lin_driver_diag_t *LinDriver_GetDiag(void);

#ifdef __cplusplus
}
#endif

#endif /* LIN_DRIVER_H */
