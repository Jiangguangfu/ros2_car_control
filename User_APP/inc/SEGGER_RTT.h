/**
 ******************************************************************************
 * @file    SEGGER_RTT.h
 * @brief   Minimal J-Link RTT (control-block compatible)
 ******************************************************************************
 */
#ifndef SEGGER_RTT_H
#define SEGGER_RTT_H

#ifdef __cplusplus
extern "C" {
#endif

#define SEGGER_RTT_MODE_NO_BLOCK_SKIP      (0)
#define SEGGER_RTT_MODE_NO_BLOCK_TRIM      (1)
#define SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL (2)

void     SEGGER_RTT_Init(void);
unsigned SEGGER_RTT_Write(unsigned buffer_index, const void *p_buffer, unsigned num_bytes);
unsigned SEGGER_RTT_WriteString(unsigned buffer_index, const char *s);
unsigned SEGGER_RTT_PutChar(unsigned buffer_index, char c);

#ifdef __cplusplus
}
#endif

#endif /* SEGGER_RTT_H */
