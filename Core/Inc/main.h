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
#include "stm32h7xx_hal.h"

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
#define RS485_1_RX_Pin GPIO_PIN_2
#define RS485_1_RX_GPIO_Port GPIOE
#define RS485_1_TX_Pin GPIO_PIN_3
#define RS485_1_TX_GPIO_Port GPIOE
#define RS485_1_RE_Pin GPIO_PIN_4
#define RS485_1_RE_GPIO_Port GPIOE
#define GPIO_OUTPUT_1_Pin GPIO_PIN_5
#define GPIO_OUTPUT_1_GPIO_Port GPIOE
#define GPIO_OUTPUT_2_Pin GPIO_PIN_6
#define GPIO_OUTPUT_2_GPIO_Port GPIOE
#define GPIO_OUTPUT_3_Pin GPIO_PIN_13
#define GPIO_OUTPUT_3_GPIO_Port GPIOC
#define GPIO_OUTPUT_5_Pin GPIO_PIN_14
#define GPIO_OUTPUT_5_GPIO_Port GPIOC
#define GPIO_OUTPUT_4_Pin GPIO_PIN_15
#define GPIO_OUTPUT_4_GPIO_Port GPIOC
#define GPIO_INPUT_1_Pin GPIO_PIN_0
#define GPIO_INPUT_1_GPIO_Port GPIOC
#define GPIO_INPUT_2_Pin GPIO_PIN_5
#define GPIO_INPUT_2_GPIO_Port GPIOA
#define GPIO_INPUT_3_Pin GPIO_PIN_6
#define GPIO_INPUT_3_GPIO_Port GPIOA
#define GPIO_INPUT_4_Pin GPIO_PIN_0
#define GPIO_INPUT_4_GPIO_Port GPIOB
#define GPIO_INPUT_5_Pin GPIO_PIN_1
#define GPIO_INPUT_5_GPIO_Port GPIOB
#define GPIO_INPUT_6_Pin GPIO_PIN_2
#define GPIO_INPUT_6_GPIO_Port GPIOB
#define GPIO_INPUT_A_Pin GPIO_PIN_9
#define GPIO_INPUT_A_GPIO_Port GPIOE
#define GPIO_INPUT_B_Pin GPIO_PIN_11
#define GPIO_INPUT_B_GPIO_Port GPIOE
#define GPIO_INPUT_Z_Pin GPIO_PIN_12
#define GPIO_INPUT_Z_GPIO_Port GPIOE
#define RS485_2_TX_Pin GPIO_PIN_10
#define RS485_2_TX_GPIO_Port GPIOB
#define ETH_RESET_Pin GPIO_PIN_14
#define ETH_RESET_GPIO_Port GPIOB
#define RS485_2_RE_Pin GPIO_PIN_8
#define RS485_2_RE_GPIO_Port GPIOD
#define RS485_2_RX_Pin GPIO_PIN_9
#define RS485_2_RX_GPIO_Port GPIOD
#define SCL_Pin GPIO_PIN_12
#define SCL_GPIO_Port GPIOD
#define SDA_Pin GPIO_PIN_13
#define SDA_GPIO_Port GPIOD
#define GPIO_INPUT_10_Pin GPIO_PIN_14
#define GPIO_INPUT_10_GPIO_Port GPIOD
#define GPIO_INPUT_9_Pin GPIO_PIN_15
#define GPIO_INPUT_9_GPIO_Port GPIOD
#define GPIO_INPUT_8_Pin GPIO_PIN_6
#define GPIO_INPUT_8_GPIO_Port GPIOC
#define GPIO_INPUT_7_Pin GPIO_PIN_7
#define GPIO_INPUT_7_GPIO_Port GPIOC
#define GPIO_INPUT_15_Pin GPIO_PIN_9
#define GPIO_INPUT_15_GPIO_Port GPIOA
#define GPIO_INPUT_14_Pin GPIO_PIN_10
#define GPIO_INPUT_14_GPIO_Port GPIOA
#define GPIO_OUTPUT_10_Pin GPIO_PIN_0
#define GPIO_OUTPUT_10_GPIO_Port GPIOD
#define GPIO_OUTPUT_9_Pin GPIO_PIN_1
#define GPIO_OUTPUT_9_GPIO_Port GPIOD
#define GPIO_OUTPUT_8_Pin GPIO_PIN_3
#define GPIO_OUTPUT_8_GPIO_Port GPIOD
#define GPIO_OUTPUT_7_Pin GPIO_PIN_4
#define GPIO_OUTPUT_7_GPIO_Port GPIOD
#define GPIO_OUTPUT_6_Pin GPIO_PIN_6
#define GPIO_OUTPUT_6_GPIO_Port GPIOD
#define GPIO_INPUT_11_Pin GPIO_PIN_7
#define GPIO_INPUT_11_GPIO_Port GPIOD
#define GPIO_INPUT_12_Pin GPIO_PIN_3
#define GPIO_INPUT_12_GPIO_Port GPIOB
#define GPIO_INPUT_13_Pin GPIO_PIN_4
#define GPIO_INPUT_13_GPIO_Port GPIOB
#define GPIO_INPUT_16_Pin GPIO_PIN_7
#define GPIO_INPUT_16_GPIO_Port GPIOB
#define GPIO_INPUT_17_Pin GPIO_PIN_8
#define GPIO_INPUT_17_GPIO_Port GPIOB
#define GPIO_INPUT_18_Pin GPIO_PIN_9
#define GPIO_INPUT_18_GPIO_Port GPIOB
#define GPIO_INPUT_19_Pin GPIO_PIN_0
#define GPIO_INPUT_19_GPIO_Port GPIOE
#define GPIO_INPUT_20_Pin GPIO_PIN_1
#define GPIO_INPUT_20_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
