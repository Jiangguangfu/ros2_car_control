#ifndef LOW_POWER_CONSUMPTION_H
#define LOW_POWER_CONSUMPTION_H

#include <stdbool.h>

#include "bq76942.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  LOW_POWER_STATE_NORMAL = 0,
  LOW_POWER_STATE_RELAX,
  LOW_POWER_STATE_SLEEP
} low_power_state_t;

/**
 * 初始化低功耗控制状态。BQ 的 Sleep Current 在 InitCalibration 中写为
 * BQ76942_SLEEP_CURRENT_MA。
 */
void LowPower_Init(void);

/**
 * 允许 BQ 根据 CC1 电流自动进入 RELAX/SLEEP。
 * 保护故障、充电器连接或 |CC1| 达到阈值时会发送 SLEEP_DISABLE。
 */
void LowPower_Process(I2C_HandleTypeDef *hi2c,
                      const bq76942_meas_t *meas,
                      bool protection_valid,
                      bool protection_fault,
                      bool charger_connected);

/** 主机显式发送 0x009A SLEEP_DISABLE，并保持禁止，直到 EnableSleep。 */
bool LowPower_DisableSleep(I2C_HandleTypeDef *hi2c);

/** 解除主机禁止；下一次 Process 满足条件时发送 0x0099 SLEEP_ENABLE。 */
void LowPower_EnableSleep(void);

low_power_state_t LowPower_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* LOW_POWER_CONSUMPTION_H */
