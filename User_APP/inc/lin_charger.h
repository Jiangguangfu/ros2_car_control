/**
 * @file    lin_charger.h
 * @brief   LIN 充电桩通信状态机（握手 / V-I 协商 / 允许电流上报 / 1s 超时暂停）
 */
#ifndef LIN_CHARGER_H
#define LIN_CHARGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "lin_charger_protocol.h"

typedef enum
{
  LIN_SESSION_IDLE = 0,
  LIN_SESSION_HANDSHAKED,
  LIN_SESSION_VI_OK,
  LIN_SESSION_ACTIVE
} lin_session_state_t;

typedef struct
{
  lin_session_state_t session;
  bool comm_lost;           /* 1 s 无 Master 帧 */
  uint16_t v_allow_mv;
  uint16_t i_allow_ma;
  uint32_t last_master_ms;
} lin_charger_status_t;

void LinCharger_Init(void);

/** 周期调用（建议 100~200 ms），检查 1 s 通信超时并更新暂停。 */
void LinCharger_Process(void);

/**
 * Master 调度帧到达（由 LIN 驱动层调用）。
 * @param pid     LIN PID
 * @param data    Data 域
 * @param len     Data 长度
 * @param rsp     从机响应缓冲（最多 8 B）
 * @param rsp_len 输出响应长度；无响应时置 0
 * @return true 表示 rsp 有效
 */
bool LinCharger_OnMasterFrame(uint8_t pid, const uint8_t *data, uint8_t len,
                              uint8_t *rsp, uint8_t *rsp_len);

const lin_charger_status_t *LinCharger_GetStatus(void);
bool LinCharger_IsCommLost(void);

#ifdef __cplusplus
}
#endif

#endif /* LIN_CHARGER_H */
