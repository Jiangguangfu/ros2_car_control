/**
 * @file    lin_charger_protocol.h
 * @brief   BMS ↔ 充电桩 LIN 协议（19200 Slave，参考 0x8B TYPE + packed 布局）
 *
 * 角色：充电桩 = LIN Master，BMS = LIN Slave（NAD 0x02）
 * 单帧 Data 最多 8 字节；字节序小端。
 */
#ifndef LIN_CHARGER_PROTOCOL_H
#define LIN_CHARGER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** BMS 从机 NAD */
#define LIN_NAD_BMS                     0x02u

/** 协议版本 */
#define LIN_PROTO_VER                   0x01u

/** 6S 包满充电压上限（mV） */
#define LIN_PACK_VMAX_MV                25200u

/** BMS 默认最大充电电流（mA） */
#define LIN_IMAX_DEFAULT_MA             3000

/** Master 帧丢失超时 → 暂停充电（ms） */
#define LIN_COMM_TIMEOUT_MS             1000u
/** 已停充暂停时放宽（J-Link 调试 halt 不致立刻 comm_lost） */
#define LIN_COMM_TIMEOUT_PAUSED_MS      3000u

/* -------------------------------------------------------------------------- */
/* LIN PID（Protected Identifier，不含校验位）                                 */
/* -------------------------------------------------------------------------- */
#define LIN_PID_CMD_HANDSHAKE           0x10u
#define LIN_PID_RSP_HANDSHAKE           0x11u
#define LIN_PID_CMD_VI_REQUEST          0x20u
#define LIN_PID_RSP_VI_ACK              0x21u
#define LIN_PID_CMD_CHARGE_CTRL         0x30u
#define LIN_PID_RSP_CHARGE_STATUS       0x31u
#define LIN_PID_CMD_STATUS_POLL         0x32u
/** Master 轮询均衡监控；BMS 应答 8 B，布局同 CAN 0x49B。 */
#define LIN_PID_CMD_BALANCE_POLL        0x33u
/** LIN 仅 6-bit ID；心跳使用 ID 63（0x3F），不可用 0xF0。 */
#define LIN_PID_CMD_HEARTBEAT           0x3Fu

/* -------------------------------------------------------------------------- */
/* Payload byte0：MSG TYPE（类比 UART 0x8B / 0x9A）                            */
/* -------------------------------------------------------------------------- */
#define LIN_MSG_HANDSHAKE_REQ           0x01u
#define LIN_MSG_HANDSHAKE_RSP           0x02u
#define LIN_MSG_VI_REQUEST              0x10u
#define LIN_MSG_VI_ACK                  0x11u
#define LIN_MSG_CHARGE_CTRL             0x20u
#define LIN_MSG_CHARGE_STATUS           0x30u
#define LIN_MSG_HEARTBEAT               0xFFu

/* 握手应答 status */
#define LIN_HS_STATUS_READY             0x00u
#define LIN_HS_STATUS_FAULT             0x01u
#define LIN_HS_STATUS_CHARGING          0x02u

/* V/I 协商 flags（Master 请求 byte5） */
#define LIN_VI_FLAG_REQUEST_START       (1u << 0)

/* V/I 协商 result（BMS 应答 byte5） */
#define LIN_VI_RESULT_ACCEPT            0x00u
#define LIN_VI_RESULT_VOLT_REJECT       0x01u
#define LIN_VI_RESULT_CURR_LIMITED      0x02u
#define LIN_VI_RESULT_FAULT             0x03u
#define LIN_VI_RESULT_THERMAL_LIMIT     0x04u

/* 充电控制 cmd（Master byte1） */
#define LIN_CHARGE_CTRL_STOP            0x00u
#define LIN_CHARGE_CTRL_START           0x01u

/* 充电状态上报 state（BMS byte1） */
#define LIN_CHG_STATUS_IDLE             0x00u
#define LIN_CHG_STATUS_CHARGING         0x01u
#define LIN_CHG_STATUS_COMPLETED        0x02u
#define LIN_CHG_STATUS_PAUSED           0x03u
#define LIN_CHG_STATUS_FAULT            0x04u

/* 充电状态上报 phase（BMS byte2） */
#define LIN_CHG_PHASE_NONE              0x00u
#define LIN_CHG_PHASE_CC                0x01u
#define LIN_CHG_PHASE_CV                0x02u

/* -------------------------------------------------------------------------- */
/* Payload 结构（packed，小端）                                                */
/* -------------------------------------------------------------------------- */

/** Master → BMS：握手请求 PID 0x10，2 B */
typedef struct __attribute__((packed)) {
  uint8_t msg_type;   /* LIN_MSG_HANDSHAKE_REQ */
  uint8_t proto_ver;
} lin_handshake_req_t;

/** BMS → Master：握手应答 PID 0x11，4 B */
typedef struct __attribute__((packed)) {
  uint8_t msg_type;       /* LIN_MSG_HANDSHAKE_RSP */
  uint8_t proto_ver;
  uint8_t series_cells;   /* 6S = 6 */
  uint8_t status;         /* LIN_HS_STATUS_* */
} lin_handshake_rsp_t;

/** Master → BMS：V/I 协商请求 PID 0x20，6 B */
typedef struct __attribute__((packed)) {
  uint8_t msg_type;       /* LIN_MSG_VI_REQUEST */
  uint16_t v_max_mv;      /* 充电桩能力电压 mV */
  uint16_t i_max_ma;      /* 充电桩能力电流 mA */
  uint8_t flags;          /* LIN_VI_FLAG_* */
} lin_vi_request_t;

/** BMS → Master：V/I 协商应答 PID 0x21，6 B */
typedef struct __attribute__((packed)) {
  uint8_t msg_type;       /* LIN_MSG_VI_ACK */
  uint16_t v_allow_mv;    /* BMS 允许电压 mV */
  uint16_t i_allow_ma;    /* BMS 允许电流 mA */
  uint8_t result;         /* LIN_VI_RESULT_* */
} lin_vi_ack_t;

/** Master → BMS：充电控制 PID 0x30，2 B */
typedef struct __attribute__((packed)) {
  uint8_t msg_type;       /* LIN_MSG_CHARGE_CTRL */
  uint8_t cmd;            /* LIN_CHARGE_CTRL_* */
} lin_charge_ctrl_t;

/** BMS → Master：充电状态 / 允许电流上报 PID 0x31 / 0x32 响应，8 B */
typedef struct __attribute__((packed)) {
  uint8_t msg_type;       /* LIN_MSG_CHARGE_STATUS */
  uint8_t state;          /* LIN_CHG_STATUS_* */
  uint8_t phase;          /* LIN_CHG_PHASE_* */
  uint16_t i_allow_ma;    /* 当前允许电流 mA */
  uint16_t pack_mv;       /* 包电压 mV */
  uint8_t fault_code;     /* charge_fault_reason_t，0=无 */
} lin_charge_status_t;

/** Master → BMS：心跳 PID 0xF0，1 B */
typedef struct __attribute__((packed)) {
  uint8_t msg_type;       /* LIN_MSG_HEARTBEAT */
} lin_heartbeat_t;

#ifdef __cplusplus
}
#endif

#endif /* LIN_CHARGER_PROTOCOL_H */
