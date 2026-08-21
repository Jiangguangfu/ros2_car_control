/**
 * @file    charge_reject.h
 * @brief   BMS 充电安全仲裁结果（ROS / CAN 0xA1 / LIN 共用）
 */
#ifndef CHARGE_REJECT_H
#define CHARGE_REJECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 主因（单值）。0 = 允许充电。 */
typedef enum
{
  CHARGE_REJECT_NONE = 0,
  CHARGE_REJECT_FULL,           /* 已满电未回落 */
  CHARGE_REJECT_MEAS,           /* BQ 测量无效 */
  CHARGE_REJECT_COMM,           /* BQ I2C 通信失败 */
  CHARGE_REJECT_OVERCURRENT,    /* OCC/OCD/SCD/软过流 */
  CHARGE_REJECT_THERMAL,        /* 过温 / 传感器故障 */
  CHARGE_REJECT_COLD,           /* 低温禁充 */
  CHARGE_REJECT_OVERVOLT,       /* 过压 / COV */
  CHARGE_REJECT_UNDERVOLT,      /* 欠压 / CUV */
  CHARGE_REJECT_IMBALANCE,      /* 压差停充 */
  CHARGE_REJECT_LIN_COMM,       /* LIN 丢帧暂停 */
  CHARGE_REJECT_LIN_NOT_READY,  /* 未握手或无桩 */
  CHARGE_REJECT_FAULT,          /* 状态机故障且不可恢复 */
  CHARGE_REJECT_BQ_PROTECT,     /* BQ Safety 锁存 */
  CHARGE_REJECT_NO_CURRENT      /* 命令已接受但超时无充电电流 */
} charge_reject_t;

#define CHG_INH_THERMAL     (1u << 0)
#define CHG_INH_PROTECT     (1u << 1)
#define CHG_INH_VOLTAGE     (1u << 2)
#define CHG_INH_IMBALANCE   (1u << 3)
#define CHG_INH_LIN_COMM    (1u << 4)
#define CHG_INH_MEAS        (1u << 5)
#define CHG_INH_COMM        (1u << 6)
#define CHG_INH_FULL        (1u << 7)
#define CHG_INH_FAULT       (1u << 8)
#define CHG_INH_LIN_SESSION (1u << 9)
#define CHG_INH_NO_CURRENT  (1u << 10)

typedef struct
{
  charge_reject_t code;
  uint16_t mask;
} charge_gate_result_t;

#ifdef __cplusplus
}
#endif

#endif /* CHARGE_REJECT_H */
