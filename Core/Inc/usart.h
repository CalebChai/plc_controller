/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.h
 * @brief   This file contains all the function prototypes for
 *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define USART2_RXBUFFERSIZE 1 // 每次接收1个数据进入一次中断
#define USART2_REC_LEN 200
#define USART10_RXBUFFERSIZE 1 // 每次接收1个数据进入一次中断
#define USART10_REC_LEN 200
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart10;

/* USER CODE BEGIN Private defines */
  extern uint8_t USART2_aRxBuffer[USART2_RXBUFFERSIZE];   // HAL库使用的串口接收缓冲
  extern uint8_t USART2_RX_BUF[USART2_REC_LEN];           // 接收缓冲,最大USART_REC_LEN个字节.
  extern uint8_t USART10_aRxBuffer[USART10_RXBUFFERSIZE]; // HAL库使用的串口接收缓冲
  extern uint8_t USART10_RX_BUF[USART10_REC_LEN];         // 接收缓冲,最大USART_REC_LEN个字节.
/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART10_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

