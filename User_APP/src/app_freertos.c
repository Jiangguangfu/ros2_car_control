/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_buzzer.h"
#include "bsp_fan.h"
#include "bq76942.h"
#include "cell_balance_manager.h"
#include "charge_path.h"
#include "charge_manager.h"
#include "cell_voltage_protect.h"
#include "bsp_power_rails.h"
#include "bsp_adc_rails.h"
#include "bms_can_tx.h"
#include "bms_can_debug.h"
#include "bms_can_ext_tx.h"
#include "bms_can_balance_tx.h"
#include "bms_balance_rtt.h"
#include "soc_estimator.h"
#include "soh_estimator.h"
#include "bms_lin_config.h"
#include "soft_start.h"
#include "uart_battery_report.h"
#include "lin_charger.h"
#include "lin_driver.h"
#if BMS_LIN_DIAG_TX_ENABLE
#include "lin_diag_tx.h"
#endif
#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMS_TASK_PERIOD_MS                500U
#define LIN_CHARGER_PROCESS_MS            100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef hi2c2;

static bq76942_temp_t s_bq_temp;
static bq76942_meas_t s_bq_meas;
static bq76942_prot_cfg_t s_bq_prot_cfg;
static uint32_t s_bq_temp_fail_count;
static uint32_t s_bq_comm_fail_count;
static uint32_t s_bq_meas_fail_count;

/* USER CODE END Variables */
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024 * 4
};
/* Definitions for ServiceTask */
osThreadId_t ServiceTaskHandle;
const osThreadAttr_t ServiceTask_attributes = {
  .name = "ServiceTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 512 * 4
};
/* Definitions for PowerTask */
osThreadId_t PowerTaskHandle;
const osThreadAttr_t PowerTask_attributes = {
  .name = "PowerTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for BmsTask */
osThreadId_t BmsTaskHandle;
const osThreadAttr_t BmsTask_attributes = {
  .name = "BmsTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1280 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
const bq76942_temp_t *Bms_GetBqTemperatures(void)
{
  return &s_bq_temp;
}

const bq76942_meas_t *Bms_GetBqMeasurements(void)
{
  return &s_bq_meas;
}

uint32_t Bms_GetBqTempFailCount(void)
{
  return s_bq_temp_fail_count;
}

uint32_t Bms_GetBqCommFailCount(void)
{
  return s_bq_comm_fail_count;
}

void Bms_RecordBqI2cResult(bool success)
{
  if (success)
  {
    s_bq_comm_fail_count = 0U;
  }
  else if (s_bq_comm_fail_count < 0xFFFFFFFFU)
  {
    s_bq_comm_fail_count++;
  }
}
uint32_t Bms_GetBqMeasFailCount(void)
{
  return s_bq_meas_fail_count;
}

const bq76942_prot_cfg_t *Bms_GetBqProtectionConfig(void)
{
  return &s_bq_prot_cfg;
}

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  BQ76942_LockInit();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommonTaskCommon, NULL, &CommTask_attributes);

  /* creation of ServiceTask */
  ServiceTaskHandle = osThreadNew(StartServiceTask, NULL, &ServiceTask_attributes);

  /* creation of PowerTask */
  PowerTaskHandle = osThreadNew(StartPowerTask, NULL, &PowerTask_attributes);

  /* creation of BmsTask */
  BmsTaskHandle = osThreadNew(StartBmsTask, NULL, &BmsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartCommonTaskCommon */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommonTaskCommon */
void StartCommonTaskCommon(void *argument)
{
  /* USER CODE BEGIN CommTask */
  (void)argument;
  /* BootSequence 已在 main 里开 12V；LIN 驱动也在 main 里尽早 Init。 */
#if !BMS_LIN_DIAG_TX_ENABLE
  LinCharger_Init();
  LinDriver_Init();
#endif
  osDelay(1500);
#if BMS_LIN_DIAG_TX_ENABLE
  LinDiagTx_Init();
#endif

  while (!SoftStart_IsSystemReady() && !SoftStart_IsBootFault())
  {
    osDelay(10);
  }

  for (;;) {
    static uint32_t s_can_elapsed_ms;

#if !BMS_LIN_DIAG_TX_ENABLE
    LinDriver_Poll();
    LinCharger_Process();
#endif
#if BMS_LIN_DIAG_TX_ENABLE
    LinDiagTx_Poll();
#endif

    s_can_elapsed_ms += LIN_CHARGER_PROCESS_MS;
    if (s_can_elapsed_ms >= BMS_CAN_BATTERY_PERIOD_MS) {
      s_can_elapsed_ms = 0U;
      BMS_CanTx_Process();
      BMS_CanExtTx_Process();
      BMS_CanBalanceTx_Process();
      BmsBalanceRtt_Process();
#if (BMS_CAN_DEBUG_ENABLE != 0)
      BMS_CanDebug_Process();
#endif
    }

    osDelay(LIN_CHARGER_PROCESS_MS);
  }
  /* USER CODE END CommTask */
}

/* USER CODE BEGIN Header_StartServiceTask */
/**
* @brief Function implementing the ServiceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServiceTask */
void StartServiceTask(void *argument)
{
  /* USER CODE BEGIN ServiceTask */
  (void)argument;

  SoftStart_Init();

  while (!SoftStart_IsSystemReady() && !SoftStart_IsBootFault())
  {
    SoftStart_Process();
    osDelay(10);
  }

  for (;;)
  {
    osDelay(1000);
  }
  /* USER CODE END ServiceTask */
}

/* USER CODE BEGIN Header_StartPowerTask */
/**
* @brief Function implementing the PowerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPowerTask */
void StartPowerTask(void *argument)
{
  /* USER CODE BEGIN PowerTask */
  (void)argument;

  while (!SoftStart_IsSystemReady() && !SoftStart_IsBootFault())
  {
    osDelay(10);
  }

  if (SoftStart_IsSystemReady())
  {
    ChargePath_Init();
    BSP_PowerRails_Init();
  }

  for (;;)
  {
    if (SoftStart_IsSystemReady())
    {
      BSP_AdcRails_Update();
      BSP_PowerRails_Process();
      ChargePath_Apply();
    }
    osDelay(200);
  }
  /* USER CODE END PowerTask */
}

/* USER CODE BEGIN Header_StartBmsTask */
/**
* @brief Function implementing the BmsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBmsTask */
void StartBmsTask(void *argument)
{
  /* USER CODE BEGIN BmsTask */
  static bool s_bq_prot_read = false;
  (void)argument;

  while (!SoftStart_IsSystemReady() && !SoftStart_IsBootFault())
  {
    osDelay(10);
  }

  if (!SoftStart_IsSystemReady())
  {
    for (;;)
    {
      osDelay(1000);
    }
  }

  ChargePath_Init();
  ChargeManager_Init();
  CellVoltageProtect_Init();
  Balance_Init();
  Soc_Init();
  Soh_Init();

  if (!SoftStart_IsBqCalibrated())
  {
    SoftStart_SetBqCalibrated(BQ76942_InitCalibration(&hi2c2));
  }

  for (;;)
  {
    if (!SoftStart_IsBqCalibrated())
    {
      SoftStart_SetBqCalibrated(BQ76942_InitCalibration(&hi2c2));
    }

    if (SoftStart_IsBqCalibrated() && !s_bq_prot_read)
    {
      s_bq_prot_read = BQ76942_ReadProtectionConfig(&hi2c2, &s_bq_prot_cfg);
    }

    if (BQ76942_ReadTemperatures(&hi2c2, &s_bq_temp))
    {
      s_bq_temp_fail_count = 0U;
    }
    else
    {
      s_bq_temp.valid = false;
      s_bq_temp_fail_count++;
    }

    if (BQ76942_ReadMeasurements(&hi2c2, &s_bq_meas))
    {
      s_bq_meas_fail_count = 0U;
    }
    else
    {
      s_bq_meas.valid = false;
      s_bq_meas_fail_count++;
    }

    Soc_Process(&s_bq_meas, BMS_TASK_PERIOD_MS);
    Balance_SetSoc(Soc_GetPercent(), Soc_IsValid());

    Balance_Process(&hi2c2);

    {
      uint8_t status_a = 0U;
      uint8_t status_b = 0U;
      uint8_t status_c = 0U;
      bool ok = BQ76942_ReadSafetyStatusEx(&hi2c2, &status_a, &status_b,
                                          &status_c);
      BSP_PowerRails_UpdateBqSafety(status_a, status_b, status_c, ok);
      CellVoltageProtect_Process(status_a, ok, &s_bq_meas);
    }

    ChargePath_Apply();
    ChargeManager_Process(&hi2c2);

    {
      const pwr_rails_status_t *pwr = BSP_PowerRails_GetStatus();
      soh_inputs_t soh_in = {
          .meas = &s_bq_meas,
          .temp = &s_bq_temp,
          .protect = pwr,
          .charge_state = ChargeManager_GetState(),
          .bq_protect = (pwr != NULL) && pwr->bq_valid && pwr->bq_any,
          .comm_fail_count = Bms_GetBqCommFailCount(),
      };
      Soh_Process(&soh_in, BMS_TASK_PERIOD_MS);
    }

    osDelay(BMS_TASK_PERIOD_MS);
  }
  /* USER CODE END BmsTask */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

