/**
 * @file    lin_charger.c
 * @brief   LIN 充电桩通信：握手、V/I 协商、允许电流上报、1 s 超时暂停充电。
 */
#include "lin_charger.h"

#include <string.h>

#include "app_freertos.h"
#include "bms_balance_snapshot.h"
#include "bsp_power_rails.h"
#include "charge_manager.h"
#include "charge_path.h"
#include "bms_can_charge_cmd.h"
#include "cmsis_os2.h"
#include "soft_start.h"
#include "uart_charge_ctrl.h"

#define LIN_SERIES_CELLS              6u
/* Safe boot target for a 1 A current-limited bench supply.  The 407 may
 * raise it later with a validated 0x441 command. */
#define LIN_INITIAL_CHARGE_MA          600u

typedef struct
{
  lin_session_state_t session;
  bool comm_lost;
  uint16_t v_allow_mv;
  uint16_t i_allow_ma;
  uint16_t target_charge_ma;
  uint32_t last_master_ms;
  bool inited;
  /* 407 STOP is a supervisory safety command.  Keep it latched so periodic
   * LIN START/VI requests from the charger cannot immediately restart. */
  bool remote_stop_latched;
} lin_charger_ctx_t;

static lin_charger_ctx_t s_lin;

static void lin_set_session(lin_session_state_t session)
{
  s_lin.session = session;
  ChargeManager_SetLinChargeExpect(session == LIN_SESSION_ACTIVE);
}

static void lin_touch_master(uint32_t now_ms)
{
  s_lin.last_master_ms = now_ms;

  if (s_lin.comm_lost)
  {
    s_lin.comm_lost = false;
    ChargePath_SetLinCommInhibit(false);
  }
}

static uint16_t lin_compute_i_limit_ma(void)
{
  const pwr_rails_status_t *pwr = BSP_PowerRails_GetStatus();
  uint16_t limit = s_lin.target_charge_ma;

  if (limit == 0U)
  {
    limit = LIN_INITIAL_CHARGE_MA;
  }

  if ((pwr != NULL) && (pwr->state >= PWR_STATE_LIMIT))
  {
    limit = (uint16_t)(limit / 2U);
  }

  if ((pwr != NULL) && (!pwr->sensor_ok))
  {
    limit = 0U;
  }

  if (s_lin.i_allow_ma > 0U)
  {
    if (limit > s_lin.i_allow_ma)
    {
      limit = s_lin.i_allow_ma;
    }
  }

  return limit;
}

static uint8_t lin_map_charge_state(void)
{
  const charge_status_t *chg = ChargeManager_GetStatus();

  if (s_lin.comm_lost &&
      (ChargeManager_GetState() == CHARGE_STATE_CHARGING))
  {
    return LIN_CHG_STATUS_PAUSED;
  }

  if (chg == NULL)
  {
    return LIN_CHG_STATUS_IDLE;
  }

  switch (chg->state)
  {
    case CHARGE_STATE_CHARGING:
      if (chg->no_current_after_ack)
      {
        return LIN_CHG_STATUS_PAUSED;
      }
      if (chg->charge_paused || ChargePath_IsLinCommInhibit())
      {
        return LIN_CHG_STATUS_PAUSED;
      }
      return LIN_CHG_STATUS_CHARGING;

    case CHARGE_STATE_COMPLETED:
      return LIN_CHG_STATUS_COMPLETED;

    case CHARGE_STATE_FAULT:
      return LIN_CHG_STATUS_FAULT;

    case CHARGE_STATE_IDLE:
    default:
      return LIN_CHG_STATUS_IDLE;
  }
}

static uint8_t lin_map_charge_phase(void)
{
  switch (ChargeManager_GetPhase())
  {
    case CHARGE_PHASE_CC:
      return LIN_CHG_PHASE_CC;

    case CHARGE_PHASE_CV:
      return LIN_CHG_PHASE_CV;

    default:
      return LIN_CHG_PHASE_NONE;
  }
}

static uint8_t lin_hs_status(void)
{
  if (ChargeManager_GetState() == CHARGE_STATE_FAULT)
  {
    return LIN_HS_STATUS_FAULT;
  }

  if (ChargeManager_GetState() == CHARGE_STATE_CHARGING)
  {
    return LIN_HS_STATUS_CHARGING;
  }

  return LIN_HS_STATUS_READY;
}

static bool lin_build_charge_status(lin_charge_status_t *out)
{
  const charge_status_t *chg = ChargeManager_GetStatus();

  if (out == NULL)
  {
    return false;
  }

  out->msg_type = LIN_MSG_CHARGE_STATUS;
  out->state = lin_map_charge_state();
  out->phase = lin_map_charge_phase();
  out->i_allow_ma = lin_compute_i_limit_ma();
  out->pack_mv = (chg != NULL) ? (uint16_t)chg->pack_mv : 0U;
  if ((chg != NULL) && chg->no_current_after_ack)
  {
    out->fault_code = (uint8_t)CHARGE_FAULT_NO_CURRENT;
  }
  else
  {
    out->fault_code = (chg != NULL) ? (uint8_t)chg->fault_reason : 0U;
  }
  return true;
}

static bool lin_handle_handshake(const uint8_t *data, uint8_t len,
                                 uint8_t *rsp, uint8_t *rsp_len)
{
  const lin_handshake_req_t *req;
  lin_handshake_rsp_t rsp_pkt;

  if ((data == NULL) || (rsp == NULL) || (rsp_len == NULL) ||
      (len < sizeof(lin_handshake_req_t)))
  {
    return false;
  }

  req = (const lin_handshake_req_t *)(const void *)data;
  if (req->msg_type != LIN_MSG_HANDSHAKE_REQ)
  {
    return false;
  }

  if (req->proto_ver != LIN_PROTO_VER)
  {
    return false;
  }

  if (ChargeManager_GetState() == CHARGE_STATE_FAULT)
  {
    (void)ChargeManager_ClearFault();
  }

  lin_set_session(LIN_SESSION_HANDSHAKED);

  rsp_pkt.msg_type = LIN_MSG_HANDSHAKE_RSP;
  rsp_pkt.proto_ver = LIN_PROTO_VER;
  rsp_pkt.series_cells = LIN_SERIES_CELLS;
  rsp_pkt.status = lin_hs_status();

  (void)memcpy(rsp, &rsp_pkt, sizeof(rsp_pkt));
  *rsp_len = (uint8_t)sizeof(rsp_pkt);
  return true;
}

static bool lin_handle_vi_request(const uint8_t *data, uint8_t len,
                                  uint8_t *rsp, uint8_t *rsp_len)
{
  const lin_vi_request_t *req;
  lin_vi_ack_t rsp_pkt;
  uint16_t v_req;
  uint16_t i_req;
  const pwr_rails_status_t *pwr;

  if ((data == NULL) || (rsp == NULL) || (rsp_len == NULL) ||
      (len < sizeof(lin_vi_request_t)) ||
      (s_lin.session < LIN_SESSION_HANDSHAKED))
  {
    return false;
  }

  req = (const lin_vi_request_t *)(const void *)data;
  if (req->msg_type != LIN_MSG_VI_REQUEST)
  {
    return false;
  }

  pwr = BSP_PowerRails_GetStatus();
  v_req = req->v_max_mv;
  i_req = req->i_max_ma;

  rsp_pkt.msg_type = LIN_MSG_VI_ACK;
  rsp_pkt.result = LIN_VI_RESULT_ACCEPT;
  rsp_pkt.v_allow_mv = v_req;
  rsp_pkt.i_allow_ma = i_req;

  if (ChargeManager_GetState() == CHARGE_STATE_FAULT)
  {
    (void)ChargeManager_ClearFault();
  }

  if (ChargeManager_GetState() == CHARGE_STATE_FAULT)
  {
    rsp_pkt.result = LIN_VI_RESULT_FAULT;
    rsp_pkt.v_allow_mv = 0U;
    rsp_pkt.i_allow_ma = 0U;
  }
  else if (v_req > LIN_PACK_VMAX_MV)
  {
    rsp_pkt.result = LIN_VI_RESULT_VOLT_REJECT;
    rsp_pkt.v_allow_mv = LIN_PACK_VMAX_MV;
  }
  else if ((pwr != NULL) &&
           ((pwr->state >= PWR_STATE_LIMIT) || (!pwr->sensor_ok)))
  {
    rsp_pkt.result = LIN_VI_RESULT_THERMAL_LIMIT;
    rsp_pkt.i_allow_ma = lin_compute_i_limit_ma();
    if (i_req > rsp_pkt.i_allow_ma)
    {
      rsp_pkt.result = LIN_VI_RESULT_CURR_LIMITED;
    }
  }
  else if (i_req > LIN_IMAX_DEFAULT_MA)
  {
    rsp_pkt.result = LIN_VI_RESULT_CURR_LIMITED;
    rsp_pkt.i_allow_ma = LIN_IMAX_DEFAULT_MA;
  }

  s_lin.v_allow_mv = rsp_pkt.v_allow_mv;
  s_lin.i_allow_ma = rsp_pkt.i_allow_ma;
  lin_set_session(LIN_SESSION_VI_OK);

  if (((req->flags & LIN_VI_FLAG_REQUEST_START) != 0U) &&
      !s_lin.remote_stop_latched)
  {
    (void)ChargeManager_Start();
    if (ChargeManager_GetState() == CHARGE_STATE_CHARGING)
    {
      lin_set_session(LIN_SESSION_ACTIVE);
    }
  }

  (void)memcpy(rsp, &rsp_pkt, sizeof(rsp_pkt));
  *rsp_len = (uint8_t)sizeof(rsp_pkt);
  return true;
}

static bool lin_handle_charge_ctrl(const uint8_t *data, uint8_t len)
{
  const lin_charge_ctrl_t *req;

  if ((data == NULL) || (len < sizeof(lin_charge_ctrl_t)) ||
      (s_lin.session < LIN_SESSION_VI_OK))
  {
    return false;
  }

  req = (const lin_charge_ctrl_t *)(const void *)data;
  if (req->msg_type != LIN_MSG_CHARGE_CTRL)
  {
    return false;
  }

  if ((req->cmd == LIN_CHARGE_CTRL_START) && !s_lin.remote_stop_latched)
  {
    if (ChargeManager_Start())
    {
      lin_set_session(LIN_SESSION_ACTIVE);
    }
  }
  else if (req->cmd == LIN_CHARGE_CTRL_STOP)
  {
    ChargeManager_Stop();
    lin_set_session(LIN_SESSION_VI_OK);
  }

  return true;
}

static bool lin_handle_status_poll(uint8_t *rsp, uint8_t *rsp_len)
{
  lin_charge_status_t status;

  if ((rsp == NULL) || (rsp_len == NULL) ||
      (s_lin.session < LIN_SESSION_HANDSHAKED))
  {
    return false;
  }

  if (!lin_build_charge_status(&status))
  {
    return false;
  }

  (void)memcpy(rsp, &status, sizeof(status));
  *rsp_len = (uint8_t)sizeof(status);
  return true;
}

static bool lin_handle_balance_poll(uint8_t *rsp, uint8_t *rsp_len)
{
  uart_battery_balance_report_t report;

  if ((rsp == NULL) || (rsp_len == NULL) ||
      (s_lin.session < LIN_SESSION_HANDSHAKED))
  {
    return false;
  }

  BmsBalanceSnapshot_Fill(&report);
  (void)memcpy(rsp, &report, sizeof(report));
  *rsp_len = (uint8_t)sizeof(report);
  return true;
}

void LinCharger_Init(void)
{
  if (s_lin.inited)
  {
    return;
  }

  lin_set_session(LIN_SESSION_IDLE);
  s_lin.comm_lost = false;
  s_lin.v_allow_mv = LIN_PACK_VMAX_MV;
  s_lin.i_allow_ma = LIN_IMAX_DEFAULT_MA;
  s_lin.target_charge_ma = LIN_INITIAL_CHARGE_MA;
  s_lin.last_master_ms = osKernelGetTickCount();
  s_lin.remote_stop_latched = false;
  s_lin.inited = true;

  ChargePath_SetLinCommInhibit(false);
}

void LinCharger_Process(void)
{
  const uint32_t now_ms = osKernelGetTickCount();

  if (!s_lin.inited)
  {
    LinCharger_Init();
  }

  /* 322538b 缓启动后，桩常在 Ready 前完成 V/I（带 START）。
   * Start() 当时失败则 session 停在 VI_OK，桩已 ACTIVE 只轮询 IDLE 并停充。
   * Ready 后在任务上下文补一次 Start，不改缓启动步骤。 */
  if (SoftStart_IsSystemReady() &&
      (s_lin.session >= LIN_SESSION_VI_OK) &&
      !s_lin.remote_stop_latched &&
      (ChargeManager_GetState() == CHARGE_STATE_IDLE))
  {
    if (ChargeManager_Start())
    {
      lin_set_session(LIN_SESSION_ACTIVE);
    }
  }

  if (s_lin.session < LIN_SESSION_ACTIVE)
  {
    return;
  }

  if (ChargeManager_GetState() != CHARGE_STATE_CHARGING)
  {
    return;
  }

  if (s_lin.comm_lost)
  {
    return;
  }

  if ((now_ms - s_lin.last_master_ms) >= LIN_COMM_TIMEOUT_MS)
  {
    s_lin.comm_lost = true;
    ChargePath_SetLinCommInhibit(true);
    ChargePath_Apply();
  }
}

bool LinCharger_OnMasterFrame(uint8_t pid, const uint8_t *data, uint8_t len,
                              uint8_t *rsp, uint8_t *rsp_len)
{
  const uint32_t now_ms = osKernelGetTickCount();
  bool has_rsp = false;

  if ((rsp == NULL) || (rsp_len == NULL))
  {
    return false;
  }

  *rsp_len = 0U;

  if (!s_lin.inited)
  {
    LinCharger_Init();
  }

  switch (pid & 0x3Fu)
  {
    case LIN_PID_CMD_HANDSHAKE:
      has_rsp = lin_handle_handshake(data, len, rsp, rsp_len);
      if (has_rsp)
      {
        lin_touch_master(now_ms);
      }
      break;

    case LIN_PID_CMD_VI_REQUEST:
      has_rsp = lin_handle_vi_request(data, len, rsp, rsp_len);
      if (has_rsp)
      {
        lin_touch_master(now_ms);
      }
      break;

    case LIN_PID_CMD_CHARGE_CTRL:
      if (lin_handle_charge_ctrl(data, len))
      {
        lin_touch_master(now_ms);
      }
      break;

    case LIN_PID_CMD_STATUS_POLL:
      has_rsp = lin_handle_status_poll(rsp, rsp_len);
      if (has_rsp)
      {
        lin_touch_master(now_ms);
      }
      break;

    case LIN_PID_CMD_BALANCE_POLL:
      has_rsp = lin_handle_balance_poll(rsp, rsp_len);
      if (has_rsp)
      {
        lin_touch_master(now_ms);
      }
      break;

    case LIN_PID_CMD_HEARTBEAT:
      if ((data != NULL) && (len >= 1U) && (data[0] == LIN_MSG_HEARTBEAT))
      {
        lin_touch_master(now_ms);
      }
      break;

    default:
      break;
  }

  return has_rsp;
}

void LinCharger_ApplyCanCommand(uint8_t cmd, uint16_t i_target_ma)
{
  if (!s_lin.inited)
  {
    LinCharger_Init();
  }

  switch (cmd)
  {
    case UART_CHARGE_CTRL_SET_CURRENT:
      if (uart_charge_current_is_valid(i_target_ma))
      {
        s_lin.target_charge_ma = i_target_ma;
      }
      break;

    case UART_CHARGE_CTRL_START:
      if (!uart_charge_current_is_valid(i_target_ma))
      {
        break;
      }
      else
      {
        s_lin.target_charge_ma = i_target_ma;
      }
      s_lin.remote_stop_latched = false;
      if (ChargeManager_GetState() == CHARGE_STATE_FAULT)
      {
        (void)ChargeManager_ClearFault();
      }
      if (ChargeManager_RequestStart(true))
      {
        lin_set_session(LIN_SESSION_ACTIVE);
      }
      BMS_CanChargeCmd_NotifyHost();
      break;

    case UART_CHARGE_CTRL_STOP:
      if (i_target_ma != 0U)
      {
        break;
      }
      s_lin.remote_stop_latched = true;
      ChargeManager_Stop();
      if (s_lin.session > LIN_SESSION_VI_OK)
      {
        lin_set_session(LIN_SESSION_VI_OK);
      }
      BMS_CanChargeCmd_NotifyHost();
      break;

    default:
      break;
  }
}

const lin_charger_status_t *LinCharger_GetStatus(void)
{
  static lin_charger_status_t view;

  view.session = s_lin.session;
  view.comm_lost = s_lin.comm_lost;
  view.v_allow_mv = s_lin.v_allow_mv;
  view.i_allow_ma = s_lin.i_allow_ma;
  view.last_master_ms = s_lin.last_master_ms;
  return &view;
}

bool LinCharger_IsCommLost(void)
{
  return s_lin.comm_lost;
}
