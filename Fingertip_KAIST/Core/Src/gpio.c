/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TOF_XSHUT_2_Pin|PR_CS_8_Pin|PR_CS_9_Pin|PR_CS_10_Pin
                          |TOF_XSHUT_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TOF_XSHUT_3_Pin|PR_CS_12_Pin|PR_CS_11_Pin|LED_1_Pin
                          |PR_CS_6_Pin|PR_CS_5_Pin|IMU_CS_Pin|IMU_NRST_Pin
                          |IMU_WAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PR_CS_4_GPIO_Port, PR_CS_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TOF_XSHUT_2_Pin PR_CS_8_Pin PR_CS_9_Pin PR_CS_10_Pin
                           TOF_XSHUT_1_Pin */
  GPIO_InitStruct.Pin = TOF_XSHUT_2_Pin|PR_CS_8_Pin|PR_CS_9_Pin|PR_CS_10_Pin
                          |TOF_XSHUT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_XSHUT_3_Pin PR_CS_12_Pin PR_CS_11_Pin LED_1_Pin
                           PR_CS_6_Pin PR_CS_5_Pin IMU_CS_Pin IMU_NRST_Pin
                           IMU_WAKE_Pin */
  GPIO_InitStruct.Pin = TOF_XSHUT_3_Pin|PR_CS_12_Pin|PR_CS_11_Pin|LED_1_Pin
                          |PR_CS_6_Pin|PR_CS_5_Pin|IMU_CS_Pin|IMU_NRST_Pin
                          |IMU_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PR_CS_4_Pin */
  GPIO_InitStruct.Pin = PR_CS_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PR_CS_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
