/**
 ******************************************************************************
 * @file    bq76942.h
 * @brief   BQ76942 I2C helpers: temperature, cells, FET, passive balance, meas.
 *
 * Schematic (BQ76942PBR):
 *   TS1/TS2 — NTC; TS3 — SW2
 *   PC13 PWR_24V_BYPASS_EN + BQ_DFETOFF low + FET_ENABLE + ALL_FETS_ON → 24V out
 ******************************************************************************
 */
#ifndef BQ76942_H
#define BQ76942_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32u3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BQ76942_MAX_CELLS                 16U

/* Datasheet / TRM: 8-bit write address 0x10 (7-bit 0x08). HAL uses 7-bit << 1. */
#define BQ76942_I2C_ADDR_HAL              0x10U

/* Direct commands */
#define BQ76942_CMD_BATTERY_STATUS        0x12U
#define BQ76942_CMD_CELL1_VOLTAGE         0x14U
#define BQ76942_CMD_STACK_VOLTAGE         0x34U
#define BQ76942_CMD_PACK_VOLTAGE          0x36U
#define BQ76942_CMD_CC2_CURRENT           0x3AU
#define BQ76942_CMD_CB_ACTIVE_CELLS       0x83U
#define BQ76942_CMD_FET_STATUS            0x7FU
#define BQ76942_CMD_INT_TEMP              0x68U
#define BQ76942_CMD_TS1_TEMP              0x70U
#define BQ76942_CMD_TS2_TEMP              0x72U
#define BQ76942_CMD_TS3_TEMP              0x74U

#define BQ76942_CMD_SAFETY_STATUS_A       0x05U
#define BQ76942_CMD_SAFETY_STATUS_B       0x06U
#define BQ76942_CMD_SAFETY_STATUS_C       0x07U

/* Subcommand mailbox */
#define BQ76942_REG_CMD_LOW               0x3EU
#define BQ76942_REG_CMD_HIGH              0x3FU
#define BQ76942_REG_DATA_START            0x40U

/* Subcommands */
#define BQ76942_SUBCMD_FET_ENABLE         0x0022U /* toggle Manufacturing[FET_EN] */
#define BQ76942_SUBCMD_ALL_FETS_ON        0x0096U
#define BQ76942_SUBCMD_ALL_FETS_OFF       0x0095U
#define BQ76942_SUBCMD_MFG_STATUS         0x0057U
#define BQ76942_SUBCMD_DASTATUS5          0x0075U /* REG18/VSS/temps + CC1/CC3 */
#define BQ76942_SUBCMD_CONFIG_UPDATE      0x0090U
#define BQ76942_SUBCMD_CONFIG_UPDATE_EXIT 0x0092U

/* Offset of CC3 Current (I2, userA) within DASTATUS5 transfer buffer. */
#define BQ76942_DASTATUS5_CC3_OFFSET      20U

/* Data memory: Calibration:V Divider Offset:Vdiv Offset (I2, userV). */
#define BQ76942_DM_Vdiv_OFFSET            0x91B2U
/* cV mode: 100 = 1 V，写入 Stack/PACK/LD 的 Vdiv Offset (0x91B2)。 */
#define BQ76942_Vdiv_OFFSET_VALUE         100

/* Stack/PACK userV unit: 1=cV(10mV), 0=mV — match DA Configuration[USER_VOLTS_CV]. */
#ifndef BQ76942_USERV_IS_CV
#define BQ76942_USERV_IS_CV               1
#endif

/* Pack topology for aggregated measurements (3..10 for BQ76942). */
#ifndef BQ76942_CELL_COUNT
#define BQ76942_CELL_COUNT                6U
#endif

#if (BQ76942_CELL_COUNT < 1U) || (BQ76942_CELL_COUNT > 10U)
#error "BQ76942_CELL_COUNT must be 1..10"
#endif

/* 0x0057 Manufacturing Status bits */
#define BQ76942_MFG_FET_EN                (1U << 4)

/* 0x7F FET Status bits (common BQ769x2 mapping) */
#define BQ76942_FETSTAT_CHG_FET           (1U << 0)
#define BQ76942_FETSTAT_PCHG_FET          (1U << 1)
#define BQ76942_FETSTAT_DSG_FET           (1U << 2)
#define BQ76942_FETSTAT_PDSG_FET          (1U << 3)

typedef struct
{
  int16_t int_temp_0p1k;
  int16_t ts1_temp_0p1k;
  int16_t ts2_temp_0p1k;
  int16_t int_temp_c_x10;
  int16_t ts1_temp_c_x10;
  int16_t ts2_temp_c_x10;
  int16_t ts3_adcin_mv;
  bool valid;
} bq76942_temp_t;

typedef struct
{
  uint16_t cell_mv[BQ76942_MAX_CELLS];
  uint16_t stack_mv;
  uint8_t cell_count;
  bool valid;
} bq76942_cells_t;

typedef struct
{
  uint16_t cell_mv[BQ76942_CELL_COUNT]; /* per-cell voltage, mV */
  uint32_t pack_mv;                     /* Stack (0x34), mV */
  uint32_t output_mv;                   /* PACK pin (0x36), mV */
  int16_t current_ma;                   /* CC2 (0x3A), mA (+ charge / - discharge) */
  int16_t current_cc3_ma;               /* CC3 avg of CC2 samples, mA (DASTATUS5) */
  uint16_t vcell_min_mv;
  uint16_t vcell_max_mv;
  bool valid;
} bq76942_meas_t;

bool BQ76942_IsReady(I2C_HandleTypeDef *hi2c);
bool BQ76942_ReadDirectU16(I2C_HandleTypeDef *hi2c, uint8_t cmd, uint16_t *raw);
bool BQ76942_ReadDirectS16(I2C_HandleTypeDef *hi2c, uint8_t cmd, int16_t *raw);
bool BQ76942_ReadTempRaw(I2C_HandleTypeDef *hi2c, uint8_t cmd, int16_t *raw);
bool BQ76942_ReadTemperatures(I2C_HandleTypeDef *hi2c, bq76942_temp_t *out);
bool BQ76942_ReadMeasurements(I2C_HandleTypeDef *hi2c, bq76942_meas_t *out);

bool BQ76942_DataMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t addr,
                             const uint8_t *data, uint8_t len);
/** Write Vdiv Offset to 0x91B2 (requires CONFIG_UPDATE). Call once at init. */
bool BQ76942_WriteVdivOffset(I2C_HandleTypeDef *hi2c, int16_t offset_userv);
/** 上电后写入 Vdiv Offset（包电压校准），成功返回 true。 */
bool BQ76942_InitCalibration(I2C_HandleTypeDef *hi2c);

bool BQ76942_SubCommandWrite(I2C_HandleTypeDef *hi2c, uint16_t subcmd);
bool BQ76942_SubCommandRead(I2C_HandleTypeDef *hi2c, uint16_t subcmd,
                            uint8_t *data, uint8_t len);
bool BQ76942_SubCommandReadU16(I2C_HandleTypeDef *hi2c, uint16_t subcmd, uint16_t *value);
bool BQ76942_ReadFetStatus(I2C_HandleTypeDef *hi2c, uint8_t *fet_status);

bool BQ76942_ReadCellVoltages(I2C_HandleTypeDef *hi2c, uint8_t cell_count,
                              bq76942_cells_t *out);
bool BQ76942_ReadPackCurrent(I2C_HandleTypeDef *hi2c, int16_t *current_ma);
bool BQ76942_ReadBatteryStatus(I2C_HandleTypeDef *hi2c, uint16_t *status);
bool BQ76942_ReadSafetyStatus(I2C_HandleTypeDef *hi2c, bool *protect_active);
bool BQ76942_SetBalanceMask(I2C_HandleTypeDef *hi2c, uint16_t mask);
bool BQ76942_ReadBalanceMask(I2C_HandleTypeDef *hi2c, uint16_t *mask);

/**
 * Release MCU DFETOFF/CFETOFF, exit FET Test mode if needed, ALL_FETS_ON.
 * Required before DSG path can supply pack 24 V output.
 */
bool BQ76942_EnableDischargePath(I2C_HandleTypeDef *hi2c);

/**
 * Ensure FET_EN + ALL_FETS_ON; CFETOFF released by charge_path.
 * Returns true when CHG FET driver reports on.
 */
bool BQ76942_EnableChargePath(I2C_HandleTypeDef *hi2c);

static inline int16_t BQ76942_Temp0p1KToCx10(int16_t temp_0p1k)
{
  return (int16_t)(temp_0p1k - 2732);
}

static inline uint8_t BQ76942_CellVoltageCmd(uint8_t cell_index)
{
  return (uint8_t)(BQ76942_CMD_CELL1_VOLTAGE + (cell_index * 2U));
}

/** Stack/PACK direct commands: userV → mV. */
static inline uint32_t BQ76942_UserVToMv(uint16_t raw_user_v)
{
#if (BQ76942_USERV_IS_CV != 0)
  return ((uint32_t)raw_user_v * 10U);
#else
  return (uint32_t)raw_user_v;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* BQ76942_H */
