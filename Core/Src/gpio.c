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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_1_RE_GPIO_Port, RS485_1_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_OUTPUT_1_Pin|GPIO_OUTPUT_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_OUTPUT_3_Pin|GPIO_OUTPUT_5_Pin|GPIO_OUTPUT_4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ETH_RESET_GPIO_Port, ETH_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_2_RE_GPIO_Port, RS485_2_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SCL_Pin|SDA_Pin|GPIO_OUTPUT_10_Pin|GPIO_OUTPUT_9_Pin
                          |GPIO_OUTPUT_8_Pin|GPIO_OUTPUT_7_Pin|GPIO_OUTPUT_6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RS485_1_RE_Pin GPIO_OUTPUT_1_Pin GPIO_OUTPUT_2_Pin */
  GPIO_InitStruct.Pin = RS485_1_RE_Pin|GPIO_OUTPUT_1_Pin|GPIO_OUTPUT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_OUTPUT_3_Pin GPIO_OUTPUT_5_Pin GPIO_OUTPUT_4_Pin */
  GPIO_InitStruct.Pin = GPIO_OUTPUT_3_Pin|GPIO_OUTPUT_5_Pin|GPIO_OUTPUT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_INPUT_1_Pin GPIO_INPUT_8_Pin GPIO_INPUT_7_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_1_Pin|GPIO_INPUT_8_Pin|GPIO_INPUT_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_INPUT_2_Pin GPIO_INPUT_3_Pin GPIO_INPUT_15_Pin GPIO_INPUT_14_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_2_Pin|GPIO_INPUT_3_Pin|GPIO_INPUT_15_Pin|GPIO_INPUT_14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_INPUT_4_Pin GPIO_INPUT_5_Pin GPIO_INPUT_6_Pin GPIO_INPUT_12_Pin
                           GPIO_INPUT_13_Pin GPIO_INPUT_16_Pin GPIO_INPUT_17_Pin GPIO_INPUT_18_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_4_Pin|GPIO_INPUT_5_Pin|GPIO_INPUT_6_Pin|GPIO_INPUT_12_Pin
                          |GPIO_INPUT_13_Pin|GPIO_INPUT_16_Pin|GPIO_INPUT_17_Pin|GPIO_INPUT_18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_INPUT_Z_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIO_INPUT_Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ETH_RESET_Pin */
  GPIO_InitStruct.Pin = ETH_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ETH_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_2_RE_Pin */
  GPIO_InitStruct.Pin = RS485_2_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_2_RE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCL_Pin SDA_Pin */
  GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_INPUT_10_Pin GPIO_INPUT_9_Pin GPIO_INPUT_11_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_10_Pin|GPIO_INPUT_9_Pin|GPIO_INPUT_11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_OUTPUT_10_Pin GPIO_OUTPUT_9_Pin GPIO_OUTPUT_8_Pin GPIO_OUTPUT_7_Pin
                           GPIO_OUTPUT_6_Pin */
  GPIO_InitStruct.Pin = GPIO_OUTPUT_10_Pin|GPIO_OUTPUT_9_Pin|GPIO_OUTPUT_8_Pin|GPIO_OUTPUT_7_Pin
                          |GPIO_OUTPUT_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_INPUT_19_Pin GPIO_INPUT_20_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_19_Pin|GPIO_INPUT_20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
