/**
 ******************************************************************************
 * @file    soft_start.h
 * @brief   BMS 上电缓启动与自检接口
 ******************************************************************************
 */
#ifndef SOFT_START_H
#define SOFT_START_H

#include <stdbool.h>
#include <stdint.h>

#include "bq76942.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  POSC_S0_SELF_CHECK = 0,
  POSC_S1_PDSG,
  POSC_S2_DSG,
  POSC_S3_12V,
  POSC_S4_OTHER,
  POSC_S5_19V,
  POSC_READY,
  POSC_FAULT
} posc_state_t;

/** 上电缓启动快照：仅由缓启动模块从芯片/硬件读入。 */
typedef struct
{
  posc_state_t state;

  bool i2c_ready;
  bool calibrated;

  uint8_t safety_a;
  uint8_t safety_b;
  uint8_t safety_c;
  bool safety_valid;

  bq76942_temp_t temp;
  bq76942_meas_t meas;

  uint8_t fet_status;
  bool fet_valid;
  uint32_t s1_elapsed_ms;

  bool rail_12v_ok;
  bool rail_19v_ok;
  bool rail_6v5_ok;
  bool rail_24v_ok; /* 本项目不用 24V，始终为 false */
} posc_snapshot_t;

void SoftStart_Init(void);
void SoftStart_Process(void);

bool SoftStart_IsSystemReady(void);
bool SoftStart_IsBootFault(void);
bool SoftStart_IsBqCalibrated(void);
void SoftStart_SetBqCalibrated(bool calibrated);

const posc_snapshot_t *SoftStart_GetSnapshot(void);
/** FET Status(0x7F) 快照；bit3=PDSG，bit2=DSG。 */
uint8_t SoftStart_GetFetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* SOFT_START_H */
