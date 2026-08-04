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
#include "thermal_manager.h"
#include "bms_can_bench.h"
#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef hi2c2;

static bq76942_temp_t s_bq_temp;
static bq76942_meas_t s_bq_meas;
static uint32_t s_bq_temp_fail_count;
static uint32_t s_bq_comm_fail_count;
static uint32_t s_bq_meas_fail_count;

/* USER CODE END Variables */
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 768 * 4
};
/* Definitions for ServiceTask */
osThreadId_t ServiceTaskHandle;
const osThreadAttr_t ServiceTask_attributes = {
  .name = "ServiceTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 384 * 4
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
  .stack_size = 768 * 4
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
  /* add mutexes, ... */
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
  /* 等 ServiceTask 电源时序完成后再发 CAN */
  osDelay(1500);

  for (;;) {
    BMS_CanBench_Process();
    osDelay(200);
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

  /* BQ FET + 电源时序（CommTask 等 1500ms 后再发 CAN） */
  HAL_GPIO_WritePin(BQ_CFETOFF_GPIO_Port, BQ_CFETOFF_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BQ_DFETOFF_GPIO_Port, BQ_DFETOFF_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PWR_24V_BYPASS_EN_GPIO_Port, PWR_24V_BYPASS_EN_Pin, GPIO_PIN_SET);
  osDelay(300);
  HAL_GPIO_WritePin(PWR_19V_EN_GPIO_Port, PWR_19V_EN_Pin, GPIO_PIN_SET);
  osDelay(300);
  HAL_GPIO_WritePin(PWR_7V5_EN_GPIO_Port, PWR_7V5_EN_Pin, GPIO_PIN_SET);
  osDelay(200);
  HAL_GPIO_WritePin(PER_12V_EN_GPIO_Port, PER_12V_EN_Pin, GPIO_PIN_SET);
  osDelay(100);

  for (;;) {
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

  /* Wait for rails + first BQ samples from BmsTask. */
  osDelay(500);
  ChargePath_Init();
  Thermal_Init();

  for (;;)
  {
    Thermal_Process();
    ChargePath_Apply();
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
  static bool s_dsg_enabled = false;
  (void)argument;

  /* Wait for power rails (ServiceTask enables 7V5/12V). */
  osDelay(300);
  ChargePath_Init();
  Balance_Init();

  for (;;)
  {
    if (BQ76942_ReadTemperatures(&hi2c2, &s_bq_temp))
    {
      s_bq_temp_fail_count = 0U;
    }
    else
    {
      s_bq_temp.valid = false;
      s_bq_temp_fail_count++;
    }

    Balance_Process(&hi2c2);
    ChargePath_Apply();

    if (BQ76942_ReadMeasurements(&hi2c2, &s_bq_meas))
    {
      s_bq_meas_fail_count = 0U;
      /* cell_mv[] mV, pack_mv/output_mv mV, current_ma=CC2, current_cc3_ma=CC3 */
    }
    else
    {
      s_bq_meas.valid = false;
      s_bq_meas_fail_count++;
    }

    /* 同时：PC13 24V Bypass + BQ DSG FET；失败则周期重试 */
    if (!s_dsg_enabled)
    {
      s_dsg_enabled = BQ76942_EnableDischargePath(&hi2c2);
      ChargePath_Apply(); /* Restore CFETOFF if imbalance/thermal request */
    }

    osDelay(500);
  }
  /* USER CODE END BmsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

