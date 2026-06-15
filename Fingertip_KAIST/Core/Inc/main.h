/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOF_XSHUT_2_Pin GPIO_PIN_0
#define TOF_XSHUT_2_GPIO_Port GPIOA
#define PR_CS_8_Pin GPIO_PIN_2
#define PR_CS_8_GPIO_Port GPIOA
#define PR_CS_9_Pin GPIO_PIN_3
#define PR_CS_9_GPIO_Port GPIOA
#define PR_CS_10_Pin GPIO_PIN_4
#define PR_CS_10_GPIO_Port GPIOA
#define TOF_XSHUT_3_Pin GPIO_PIN_0
#define TOF_XSHUT_3_GPIO_Port GPIOB
#define PR_CS_12_Pin GPIO_PIN_1
#define PR_CS_12_GPIO_Port GPIOB
#define PR_CS_11_Pin GPIO_PIN_2
#define PR_CS_11_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_11
#define LED_1_GPIO_Port GPIOB
#define PR_CS_6_Pin GPIO_PIN_14
#define PR_CS_6_GPIO_Port GPIOB
#define PR_CS_5_Pin GPIO_PIN_15
#define PR_CS_5_GPIO_Port GPIOB
#define PR_CS_4_Pin GPIO_PIN_6
#define PR_CS_4_GPIO_Port GPIOC
#define TOF_XSHUT_1_Pin GPIO_PIN_15
#define TOF_XSHUT_1_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_10
#define IMU_INT_GPIO_Port GPIOC
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOB
#define IMU_NRST_Pin GPIO_PIN_6
#define IMU_NRST_GPIO_Port GPIOB
#define IMU_WAKE_Pin GPIO_PIN_7
#define IMU_WAKE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
