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
#include "bsp_ws2812.h"
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BmsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

