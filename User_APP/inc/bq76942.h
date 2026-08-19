/**
 ******************************************************************************
 * @file    bq76942.h
 * @brief   BQ76942 I2C helpers: temperature, cells, FET, passive balance, meas.
 *
 * Schematic (BQ76942PBR):
 *   TS1 — NTC (protect); TS2 — NTC (report only); TS3 — SW2
 *   PC13 PWR_24V_BYPASS_EN 本项目不用，始终关闭；BQ_DFETOFF / FET 由 charge_path 管理
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
#define BQ76942_CMD_LD_VOLTAGE            0x38U
#define BQ76942_CMD_CC2_CURRENT           0x3AU
/* Cell balancing subcommands (TRM §10; not direct-command 0x83). */
#define BQ76942_SUBCMD_CB_ACTIVE_CELLS    0x0083U
#define BQ76942_SUBCMD_CB_SET_LVL         0x0084U
#define BQ76942_SUBCMD_CBSTATUS1          0x0085U
#define BQ76942_CMD_FET_STATUS            0x7FU
#define BQ76942_CMD_INT_TEMP              0x68U
#define BQ76942_CMD_TS1_TEMP              0x70U
#define BQ76942_CMD_TS2_TEMP              0x72U
#define BQ76942_CMD_TS3_TEMP              0x74U

#define BQ76942_CMD_SAFETY_STATUS_A       0x03U
#define BQ76942_CMD_SAFETY_STATUS_B       0x05U
#define BQ76942_CMD_SAFETY_STATUS_C       0x07U

/* Subcommand mailbox */
#define BQ76942_REG_CMD_LOW               0x3EU
#define BQ76942_REG_CMD_HIGH              0x3FU
#define BQ76942_REG_DATA_START            0x40U

/* Subcommands */
#define BQ76942_SUBCMD_FET_ENABLE         0x0022U /* toggle Manufacturing[FET_EN] */
#define BQ76942_SUBCMD_ALL_FETS_ON        0x0096U
#define BQ76942_SUBCMD_ALL_FETS_OFF       0x0095U
#define BQ76942_SUBCMD_DSG_PDSG_OFF       0x0093U
#define BQ76942_SUBCMD_FET_CONTROL        0x0097U
#define BQ76942_SUBCMD_MFG_STATUS         0x0057U
#define BQ76942_SUBCMD_DASTATUS5          0x0075U /* REG18/VSS/temps + CC1/CC3 */
#define BQ76942_SUBCMD_CONFIG_UPDATE      0x0090U
#define BQ76942_SUBCMD_CONFIG_UPDATE_EXIT 0x0092U

/* Offset of CC3 Current (I2, userA) within DASTATUS5 transfer buffer. */
#define BQ76942_DASTATUS5_CC3_OFFSET      20U

/* Data memory: Calibration:Current (IEEE-754 F4, little-endian). */
#define BQ76942_DM_CC_GAIN                0x91A8U
#define BQ76942_DM_CAPACITY_GAIN          0x91ACU
/* CC Gain = 7.4768 / (Rsense_mOhm); Capacity Gain = CC Gain × 298261.6178. */
#define BQ76942_CC_GAIN_RSENSE_FACTOR     7.4768f
#define BQ76942_CAPACITY_GAIN_FACTOR      298261.6178f

/* Data memory: Settings:Protection (TRM 13.3.3). */
#define BQ76942_DM_ENABLED_PROT_A         0x9261U /* U1, default 0x88 */
#define BQ76942_DM_CHG_FET_PROT_A         0x9265U /* U1, default 0x98 */
#define BQ76942_PROT_A_SCD                (1U << 7)
#define BQ76942_PROT_A_OCD2               (1U << 6)
#define BQ76942_PROT_A_OCD1               (1U << 5)
#define BQ76942_PROT_A_OCC                (1U << 4)
#define BQ76942_PROT_A_COV                (1U << 3)
#define BQ76942_PROT_A_CUV                (1U << 2)
#ifndef BQ76942_ENABLED_PROT_A
/* 0x88 (SCD+COV) + OCC + OCD1 + OCD2 + CUV. */
#define BQ76942_ENABLED_PROT_A            (0x88U | BQ76942_PROT_A_OCC | \
                                             BQ76942_PROT_A_OCD1 | BQ76942_PROT_A_OCD2 | \
                                             BQ76942_PROT_A_CUV)
#endif
#ifndef BQ76942_CHG_FET_PROT_A
#define BQ76942_CHG_FET_PROT_A            0x98U /* OCC+COV+SCD shut CHG FET (TRM default) */
#endif
#define BQ76942_DM_DSG_FET_PROT_A         0x9269U
#ifndef BQ76942_DSG_FET_PROT_A
#define BQ76942_DSG_FET_PROT_A            0xE4U /* SCD+OCD1+OCD2 shut DSG FET (TRM default) */
#endif

/* Data memory: Protections:CUV/COV (TRM 13.6.1–13.6.2). Threshold U1 in 50.6 mV. */
#define BQ76942_DM_CUV_THRESHOLD          0x9275U
#define BQ76942_DM_COV_THRESHOLD          0x9278U
#define BQ76942_CELL_THRESHOLD_MV_FACTOR  506U  /* 50.6 mV ×10 for integer math */

/* Data memory: Protections:OCC (TRM 13.6.4). Threshold U1 in 2 mV across sense. */
#define BQ76942_DM_OCC_THRESHOLD          0x9280U
#define BQ76942_DM_OCC_DELAY              0x9281U /* U1, 3.3 ms × (2+N) */
#define BQ76942_DM_OCD1_THRESHOLD         0x9282U /* U1, units of 2 mV */
#define BQ76942_DM_OCD1_DELAY             0x9283U
#define BQ76942_DM_OCD2_THRESHOLD         0x9284U /* U1, units of 2 mV */
#define BQ76942_DM_OCD2_DELAY             0x9285U
#define BQ76942_DM_SCD_THRESHOLD          0x9286U /* U1, discrete mV table 0..15 */
#define BQ76942_DM_PROT_BLOCK_LEN         8U      /* 0x9280..0x9287 contiguous */
#ifndef BQ76942_OCC_THRESHOLD
#define BQ76942_OCC_THRESHOLD             0x02U /* 2×2 mV = 4 mV → 4 A @ 1 mΩ */
#endif
#ifndef BQ76942_OCC_DELAY
#define BQ76942_OCC_DELAY                 0x04U /* ≈20 ms */
#endif
#ifndef BQ76942_OCD1_THRESHOLD
#define BQ76942_OCD1_THRESHOLD            0x04U /* TRM default, 8 mV */
#endif
#ifndef BQ76942_OCD1_DELAY
#define BQ76942_OCD1_DELAY                0x01U /* TRM default, ≈10 ms */
#endif
#ifndef BQ76942_OCD2_THRESHOLD
#define BQ76942_OCD2_THRESHOLD            0x03U /* TRM default, 6 mV */
#endif
#ifndef BQ76942_OCD2_DELAY
#define BQ76942_OCD2_DELAY                0x07U /* TRM default */
#endif
#ifndef BQ76942_SCD_THRESHOLD
#define BQ76942_SCD_THRESHOLD             0x02U /* TRM table index 2 = 40 mV */
#endif

/* Data memory: Protections:SCD:Delay (U1, 1–31 → (N-1)×15µs). */
#define BQ76942_DM_SCD_DELAY              0x9287U
#ifndef BQ76942_SCD_DELAY
#define BQ76942_SCD_DELAY                 0x05U /* (5-1)×15µs = 60µs */
#endif

/* Data memory: Calibration:V Divider Offset:Vdiv Offset (I2, userV). */
#define BQ76942_DM_Vdiv_OFFSET            0x91B2U
/* Settings:Configuration:TS2 Config (H1). TRM 13.3.2.13 @ 0x92FE. */
#define BQ76942_DM_TS2_CONFIG             0x92FEU
/* Settings:Configuration:Vcell Mode (H2 @ 0x9304). Bit N = Cell(N+1) connected. */
#define BQ76942_DM_VCELL_MODE             0x9304U

/* Settings:Configuration: CFETOFF/DFETOFF pin (TRM 13.3.2.9/10). */
#define BQ76942_DM_CFETOFF_PIN_CONFIG     0x92FAU
#define BQ76942_DM_DFETOFF_PIN_CONFIG     0x92FBU
/* PIN_FXN=2：CFETOFF/DFETOFF 输入；OPT5=0 高有效（与 MCU SET=禁止 一致）。 */
#define BQ76942_PINCFG_FETOFF_ACTIVE_HIGH  0x02U

/* Settings:FET (TRM 13.3.6). */
#define BQ76942_DM_FET_OPTIONS            0x9308U /* default 0x0D, PDSG_EN=0 */
#define BQ76942_DM_PREDISCHARGE_TIMEOUT     0x930EU /* U1, ×10 ms; 0=voltage only */
#define BQ76942_DM_PREDISCHARGE_STOP_DELTA  0x930FU /* U1, ×10 mV */
#define BQ76942_FETOPT_PDSG_EN              (1U << 4)
#ifndef BQ76942_PREDISCHARGE_TIMEOUT
#define BQ76942_PREDISCHARGE_TIMEOUT        50U /* 50 × 10 ms = 500 ms */
#endif
#ifndef BQ76942_PREDISCHARGE_STOP_DELTA
#define BQ76942_PREDISCHARGE_STOP_DELTA     50U /* 50 × 10 mV = 500 mV */
#endif

/* 0x0097 FET_CONTROL: bit0=DSG_OFF, bit1=PDSG_OFF, bit2=CHG_OFF, bit3=PCHG_OFF。 */
#define BQ76942_FETCTRL_ALLOW_ALL           0x00U
/** 仅允许 PDSG：CHG/PCHG/DSG 强制关，PDSG 由 PDSG_EN 自动拉起。 */
#define BQ76942_FETCTRL_PDSG_ONLY           0x0DU
#define BQ76942_FETSTAT_CHG_SIDE            \
  (BQ76942_FETSTAT_CHG_FET | BQ76942_FETSTAT_PCHG_FET)
#define BQ76942_FETSTAT_DSG_SIDE            \
  (BQ76942_FETSTAT_DSG_FET | BQ76942_FETSTAT_PDSG_FET)
#ifndef BQ76942_VCELL_MODE
/* 6S: Cell1..6 (bit0..5); Cell7..10 (bit6..9) unused. */
#define BQ76942_VCELL_MODE                0x003FU
#endif
/*
 * TS2: 18k pullup + 18K thermistor model + report-only (no cell/FET protect).
 * PIN_FXN=3 (thermistor). Same as TS1 0x07 but OPT[1:0]=10 instead of 01.
 */
#define BQ76942_TS2_CONFIG_REPORT_ONLY    0x0BU
/* cV mode: 100 = 1 V，写入 Stack/PACK/LD 的 Vdiv Offset (0x91B2)。 */
#define BQ76942_Vdiv_OFFSET_VALUE         100

/* 采样电阻 (mΩ)；实测电流偏大约 1.58 倍时 CC Gain 再除以该比值。 */
#ifndef BQ76942_SENSE_RESISTOR_MOHM
#define BQ76942_SENSE_RESISTOR_MOHM       1.0f
#endif
#ifndef BQ76942_CC_GAIN_MEASURED_RATIO
#define BQ76942_CC_GAIN_MEASURED_RATIO    1.58f/*实测CC2电流与实际电流的比值*/
#endif

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

/* 0x7F FET Status — TRM 12.2.20 (bits 6:4 = pin mirrors, 3:0 = FET drivers). */
#define BQ76942_FETSTAT_CHG_FET           (1U << 0)
#define BQ76942_FETSTAT_PCHG_FET          (1U << 1)
#define BQ76942_FETSTAT_DSG_FET           (1U << 2)
#define BQ76942_FETSTAT_PDSG_FET          (1U << 3)
#define BQ76942_FETSTAT_DCHG_PIN          (1U << 4)
#define BQ76942_FETSTAT_DDSG_PIN          (1U << 5)
#define BQ76942_FETSTAT_ALRT_PIN          (1U << 6)

/* 0x03 Safety Status A — TRM (current / cell voltage faults) */
#define BQ76942_SA_SCD                    (1U << 7) /* 放电短路安全警报 */
#define BQ76942_SA_OCD2                   (1U << 6) /* 放电过流2安全警报 --6mv*/
#define BQ76942_SA_OCD1                   (1U << 5) /* 放电过流1安全警报 --8mv*/
#define BQ76942_SA_OCC                    (1U << 4) /* 充电过流安全警报 --4mv */
#define BQ76942_SA_COV                    (1U << 3) /*电芯过压安全警报*/
#define BQ76942_SA_CUV                    (1U << 2) /*电芯欠压安全警报*/

/** BQ76942 data memory protection thresholds/delays read from chip. */
typedef struct
{
  uint8_t enabled_prot_a;
  bool occ_enabled;
  bool ocd1_enabled;
  bool ocd2_enabled;
  bool scd_enabled;
  bool cov_enabled;
  bool cuv_enabled;

  uint8_t cuv_threshold_code;
  uint8_t cov_threshold_code;
  uint8_t occ_threshold_code;
  uint8_t occ_delay_code;
  uint8_t ocd1_threshold_code;
  uint8_t ocd1_delay_code;
  uint8_t ocd2_threshold_code;
  uint8_t ocd2_delay_code;
  uint8_t scd_threshold_code;
  uint8_t scd_delay_code;

  uint16_t cuv_threshold_mv;   /* code × 50.6 mV */
  uint16_t cov_threshold_mv;   /* code × 50.6 mV */
  uint16_t occ_threshold_mv;
  uint16_t ocd1_threshold_mv;
  uint16_t ocd2_threshold_mv;
  uint16_t scd_threshold_mv;

  uint16_t occ_delay_ms_x10;   /* 3.3 ms × (2 + code) */
  uint16_t ocd1_delay_ms_x10;
  uint16_t ocd2_delay_ms_x10;
  uint16_t scd_delay_us;       /* (code - 1) × 15 µs */

  uint16_t occ_trip_ma;        /* @ BQ76942_SENSE_RESISTOR_MOHM */
  uint16_t ocd1_trip_ma;
  uint16_t ocd2_trip_ma;
  uint16_t scd_trip_ma;

  bool valid;
} bq76942_prot_cfg_t;

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
  uint32_t pack_mv;                     /* Stack (0x34), 电池组顶部的 16 位电压mV */
  uint32_t output_mv;                   /* PACK pin (0x36),PACK 引脚上的 16 位电压 mV */
  uint32_t ld_mv;                       /* LD pin (0x38), 负载侧/母线电容电压 mV */
  int16_t current_ma;                   /* CC2 (0x3A), mA (+ charge / - discharge) */
  int16_t current_cc3_ma;               /* CC3 avg of CC2 samples, mA (DASTATUS5) */
  uint16_t vcell_min_mv;
  uint16_t vcell_max_mv;
  bool valid;
} bq76942_meas_t;

/** 创建 BQ I2C 递归互斥锁，须在任务启动前调用。 */
void BQ76942_LockInit(void);
/** 占用 I2C：自检/多步读写期间持有，防止其他任务交错篡改。 */
bool BQ76942_I2cLock(void);
void BQ76942_I2cUnlock(void);

bool BQ76942_IsReady(I2C_HandleTypeDef *hi2c);
bool BQ76942_ReadDirectU16(I2C_HandleTypeDef *hi2c, uint8_t cmd, uint16_t *raw);
bool BQ76942_ReadDirectS16(I2C_HandleTypeDef *hi2c, uint8_t cmd, int16_t *raw);
bool BQ76942_ReadTempRaw(I2C_HandleTypeDef *hi2c, uint8_t cmd, int16_t *raw);
bool BQ76942_ReadTemperatures(I2C_HandleTypeDef *hi2c, bq76942_temp_t *out);
bool BQ76942_ReadMeasurements(I2C_HandleTypeDef *hi2c, bq76942_meas_t *out);
/** 仅读 Stack(0x34)+PACK(0x36)，供 S1 判断输出电容是否充满。 */
bool BQ76942_ReadStackOutputMv(I2C_HandleTypeDef *hi2c, uint32_t *stack_mv,
                               uint32_t *output_mv);

bool BQ76942_DataMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t addr,
                             const uint8_t *data, uint8_t len);
/** Read data memory: write addr to 0x3E, read len bytes from 0x40. */
bool BQ76942_DataMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t addr,
                            uint8_t *data, uint8_t len);
/** Read OCC/OCD/SCD threshold + delay from chip data memory. */
bool BQ76942_ReadProtectionConfig(I2C_HandleTypeDef *hi2c,
                                  bq76942_prot_cfg_t *out);
/** Write Vdiv Offset to 0x91B2 (requires CONFIG_UPDATE). Call once at init. */
bool BQ76942_WriteVdivOffset(I2C_HandleTypeDef *hi2c, int16_t offset_userv);
/** CONFIG_UPDATE 内写入 Vdiv Offset = 1V（BQ76942_Vdiv_OFFSET_VALUE），供自检读 PACK 前调用。 */
bool BQ76942_ApplyVdivOffset1V(I2C_HandleTypeDef *hi2c);
/** Write CC Gain (0x91A8) + Capacity Gain (0x91AC); requires CONFIG_UPDATE. */
bool BQ76942_WriteCcGain(I2C_HandleTypeDef *hi2c, float cc_gain);
/** Write Protections:SCD:Delay @ 0x9287; requires CONFIG_UPDATE. */
bool BQ76942_WriteScdDelay(I2C_HandleTypeDef *hi2c, uint8_t delay_code);
/** Write OCC/OCD/SCD protection enable + thresholds/delays; requires CONFIG_UPDATE. */
bool BQ76942_WriteProtectionConfig(I2C_HandleTypeDef *hi2c);
/** 上电后写入 Vdiv Offset + CC Gain + 保护配置，成功返回 true。 */
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
/** Read Safety Status A/B/C raw bytes (0x03 / 0x05 / 0x07). */
bool BQ76942_ReadSafetyStatusEx(I2C_HandleTypeDef *hi2c,
                                uint8_t *status_a, uint8_t *status_b,
                                uint8_t *status_c);
bool BQ76942_SetBalanceMask(I2C_HandleTypeDef *hi2c, uint16_t mask);
bool BQ76942_ReadBalanceMask(I2C_HandleTypeDef *hi2c, uint16_t *mask);
/** CBSTATUS1: continuous balancing time in seconds (0x0085 read). */
bool BQ76942_ReadBalanceActiveSec(I2C_HandleTypeDef *hi2c, uint16_t *seconds);

/**
 * FET_EN + ALL_FETS_OFF + 先 PDSG_ONLY，见到 PDSG/DSG 后再 ALLOW_ALL。
 * PDSG_EN / 预放电超时已在 InitCalibration 写入。
 * 见到 PDSG 或 DSG 置位才返回 true。
 */
bool BQ76942_EnablePreDischargePath(I2C_HandleTypeDef *hi2c);

/**
 * FET_EN + ALL_FETS_ON。不要求此刻已是 DSG（芯片可能仍在 PDSG）。
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
