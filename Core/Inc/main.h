/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWR_24V_BYPASS_EN_Pin GPIO_PIN_13
#define PWR_24V_BYPASS_EN_GPIO_Port GPIOC
#define BQ_DCHG_Pin GPIO_PIN_14
#define BQ_DCHG_GPIO_Port GPIOC
#define BQ_DDSG_Pin GPIO_PIN_15
#define BQ_DDSG_GPIO_Port GPIOC
#define BUZZER_PWM_Pin GPIO_PIN_10
#define BUZZER_PWM_GPIO_Port GPIOB
#define BQ_ALERT_Pin GPIO_PIN_12
#define BQ_ALERT_GPIO_Port GPIOB
#define BQ_ALERT_EXTI_IRQn EXTI12_IRQn
#define PWR_7V5_EN_Pin GPIO_PIN_15
#define PWR_7V5_EN_GPIO_Port GPIOB
#define PER_12V_EN_Pin GPIO_PIN_8
#define PER_12V_EN_GPIO_Port GPIOA
#define FDCAN_RX_Pin GPIO_PIN_11
#define FDCAN_RX_GPIO_Port GPIOA
#define FDCAN_TX_Pin GPIO_PIN_12
#define FDCAN_TX_GPIO_Port GPIOA
#define BQ_CFETOFF_Pin GPIO_PIN_15
#define BQ_CFETOFF_GPIO_Port GPIOA
#define BQ_DFETOFF_Pin GPIO_PIN_3
#define BQ_DFETOFF_GPIO_Port GPIOB
#define PWR_19V_EN_Pin GPIO_PIN_4
#define PWR_19V_EN_GPIO_Port GPIOB
#define WS2812_DATA_Pin GPIO_PIN_3
#define WS2812_DATA_GPIO_Port GPIOH
#define SYS_FAN_PWM_Pin GPIO_PIN_8
#define SYS_FAN_PWM_GPIO_Port GPIOB
#define SYS_FAN_SPD_Pin GPIO_PIN_9
#define SYS_FAN_SPD_GPIO_Port GPIOB
#define SYS_FAN_SPD_EXTI_IRQn EXTI9_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
