/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.h
 * @brief   This file contains all the function prototypes for
 *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "semphr.h"
  /* USER CODE END Includes */

  extern FDCAN_HandleTypeDef hfdcan2;

  /* USER CODE BEGIN Private defines */
  extern FDCAN_HandleTypeDef hfdcan2;
  extern FDCAN_TxHeaderTypeDef fdcan2_TxHeader;
  extern FDCAN_RxHeaderTypeDef fdcan2_RxHeader;
  extern SemaphoreHandle_t xCanTxMutex; // ȫ�ֻ�����

  /* USER CODE END Private defines */

  void MX_FDCAN2_Init(void);

  /* USER CODE BEGIN Prototypes */
  uint8_t FDCAN2_Send_Msg(uint8_t *msg, uint32_t len, uint16_t Identifier);
  uint8_t FDCAN2_Receive_Msg(uint8_t *buf, uint16_t *Identifier);
  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
