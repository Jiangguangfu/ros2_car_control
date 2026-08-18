/**
 ******************************************************************************
 * @file    SEGGER_RTT.c
 * @brief   Minimal J-Link RTT up-channel (Terminal)
 *
 * J-Link / RTT Viewer 扫描 RAM 中的 "SEGGER RTT" 控制块；本实现布局与官方一致。
 ******************************************************************************
 */
#include "SEGGER_RTT.h"

#include <stdint.h>
#include <string.h>

#define SEGGER_RTT_MAX_NUM_UP_BUFFERS   1
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS 1
#define BUFFER_SIZE_UP                  1024U
#define BUFFER_SIZE_DOWN                16U
#define SEGGER_RTT_MODE_DEFAULT         SEGGER_RTT_MODE_NO_BLOCK_TRIM

typedef struct
{
  const char *sName;
  char *pBuffer;
  unsigned SizeOfBuffer;
  unsigned WrOff;
  volatile unsigned RdOff;
  unsigned Flags;
} SEGGER_RTT_BUFFER_UP;

typedef struct
{
  const char *sName;
  char *pBuffer;
  unsigned SizeOfBuffer;
  volatile unsigned WrOff;
  unsigned RdOff;
  unsigned Flags;
} SEGGER_RTT_BUFFER_DOWN;

typedef struct
{
  char acID[16];
  int MaxNumUpBuffers;
  int MaxNumDownBuffers;
  SEGGER_RTT_BUFFER_UP aUp[SEGGER_RTT_MAX_NUM_UP_BUFFERS];
  SEGGER_RTT_BUFFER_DOWN aDown[SEGGER_RTT_MAX_NUM_DOWN_BUFFERS];
} SEGGER_RTT_CB;

static char s_up_buffer[BUFFER_SIZE_UP];
static char s_down_buffer[BUFFER_SIZE_DOWN];

/* 放在 RAM 起始段，便于调试器尽快搜到控制块 */
__attribute__((section(".rtt"), used, aligned(16)))
SEGGER_RTT_CB _SEGGER_RTT;

static volatile unsigned char s_rtt_inited;

static void Rtt_Dmb(void)
{
  __asm volatile("dmb" ::: "memory");
}

static void Rtt_DoInit(void)
{
  unsigned i;

  (void)memset(&_SEGGER_RTT, 0, sizeof(_SEGGER_RTT));
  _SEGGER_RTT.MaxNumUpBuffers = SEGGER_RTT_MAX_NUM_UP_BUFFERS;
  _SEGGER_RTT.MaxNumDownBuffers = SEGGER_RTT_MAX_NUM_DOWN_BUFFERS;

  _SEGGER_RTT.aUp[0].sName = "Terminal";
  _SEGGER_RTT.aUp[0].pBuffer = s_up_buffer;
  _SEGGER_RTT.aUp[0].SizeOfBuffer = BUFFER_SIZE_UP;
  _SEGGER_RTT.aUp[0].Flags = SEGGER_RTT_MODE_DEFAULT;

  _SEGGER_RTT.aDown[0].sName = "Terminal";
  _SEGGER_RTT.aDown[0].pBuffer = s_down_buffer;
  _SEGGER_RTT.aDown[0].SizeOfBuffer = BUFFER_SIZE_DOWN;
  _SEGGER_RTT.aDown[0].Flags = SEGGER_RTT_MODE_DEFAULT;

  /* 最后写 ID，避免调试器附着到未完成的控制块 */
  static const char id[] = "SEGGER RTT";
  for (i = sizeof(id); i > 0U; i--)
  {
    _SEGGER_RTT.acID[i - 1U] = id[i - 1U];
  }

  Rtt_Dmb();
  s_rtt_inited = 1U;
}

void SEGGER_RTT_Init(void)
{
  Rtt_DoInit();
}

static unsigned Rtt_AvailWrite(const SEGGER_RTT_BUFFER_UP *p_up)
{
  unsigned rd = p_up->RdOff;
  unsigned wr = p_up->WrOff;

  if (rd <= wr)
  {
    return (p_up->SizeOfBuffer - 1U) - (wr - rd);
  }
  return (rd - wr) - 1U;
}

unsigned SEGGER_RTT_Write(unsigned buffer_index, const void *p_buffer, unsigned num_bytes)
{
  SEGGER_RTT_BUFFER_UP *p_up;
  const uint8_t *src;
  unsigned avail;
  unsigned wr;
  unsigned n;
  unsigned chunk;

  if ((s_rtt_inited == 0U) || (_SEGGER_RTT.acID[0] == '\0'))
  {
    Rtt_DoInit();
  }

  if ((buffer_index >= (unsigned)SEGGER_RTT_MAX_NUM_UP_BUFFERS) ||
      (p_buffer == NULL) || (num_bytes == 0U))
  {
    return 0U;
  }

  p_up = &_SEGGER_RTT.aUp[buffer_index];
  avail = Rtt_AvailWrite(p_up);
  if (avail == 0U)
  {
    return 0U;
  }

  if (num_bytes > avail)
  {
    if ((p_up->Flags & 3U) == SEGGER_RTT_MODE_NO_BLOCK_SKIP)
    {
      return 0U;
    }
    num_bytes = avail;
  }

  src = (const uint8_t *)p_buffer;
  wr = p_up->WrOff;
  n = num_bytes;

  while (n > 0U)
  {
    chunk = p_up->SizeOfBuffer - wr;
    if (chunk > n)
    {
      chunk = n;
    }
    (void)memcpy(&p_up->pBuffer[wr], src, chunk);
    wr += chunk;
    if (wr >= p_up->SizeOfBuffer)
    {
      wr = 0U;
    }
    src += chunk;
    n -= chunk;
  }

  Rtt_Dmb();
  p_up->WrOff = wr;
  Rtt_Dmb();
  return num_bytes;
}

unsigned SEGGER_RTT_WriteString(unsigned buffer_index, const char *s)
{
  unsigned len = 0U;

  if (s == NULL)
  {
    return 0U;
  }

  while (s[len] != '\0')
  {
    len++;
  }

  return SEGGER_RTT_Write(buffer_index, s, len);
}

unsigned SEGGER_RTT_PutChar(unsigned buffer_index, char c)
{
  return SEGGER_RTT_Write(buffer_index, &c, 1U);
}
