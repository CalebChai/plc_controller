/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.c
 * @brief   This file provides code for the configuration
 *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */

FDCAN_FilterTypeDef FDCAN2_RXFilter;
FDCAN_TxHeaderTypeDef fdcan2_TxHeader;
FDCAN_RxHeaderTypeDef fdcan2_RxHeader;
SemaphoreHandle_t xCanTxMutex; // 全局互斥量
int fdcan2StartFlag = 0;

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan2;

/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */
  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 11;
  hfdcan2.Init.NominalSyncJumpWidth = 4;
  hfdcan2.Init.NominalTimeSeg1 = 11;
  hfdcan2.Init.NominalTimeSeg2 = 8;
  hfdcan2.Init.DataPrescaler = 11;
  hfdcan2.Init.DataSyncJumpWidth = 6;
  hfdcan2.Init.DataTimeSeg1 = 13;
  hfdcan2.Init.DataTimeSeg2 = 6;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 4;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  // 配置RX滤波器
  FDCAN2_RXFilter.IdType = FDCAN_STANDARD_ID;                       // 标准ID
  FDCAN2_RXFilter.FilterIndex = 0;                                  // 滤波器索引
  FDCAN2_RXFilter.FilterType = FDCAN_FILTER_MASK;                   // 滤波器类型
  FDCAN2_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           // 过滤器0关联到FIFO0
  FDCAN2_RXFilter.FilterID1 = 0x00000000;                           // 32位ID
  FDCAN2_RXFilter.FilterID2 = 0x00000000;                           // 如果FDCAN配置为传统模式的话，这里是32位掩码
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_RXFilter) != HAL_OK) // 滤波器初始化
    Error_Handler();
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_Start(&hfdcan2); // 开启FDCAN
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  /* USER CODE END FDCAN2_Init 2 */
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if (fdcanHandle->Instance == FDCAN2)
  {
    /* USER CODE BEGIN FDCAN2_MspInit 0 */

    /* USER CODE END FDCAN2_MspInit 0 */

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    /* USER CODE BEGIN FDCAN2_MspInit 1 */

    /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{

  if (fdcanHandle->Instance == FDCAN2)
  {
    /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

    /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

    /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t FDCAN2_Send_Msg(uint8_t *msg, uint32_t len, uint16_t Identifier)
{
  fdcan2_TxHeader.Identifier = Identifier;        // 32位ID
  fdcan2_TxHeader.IdType = FDCAN_STANDARD_ID;     // 标准ID
  fdcan2_TxHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
  fdcan2_TxHeader.DataLength = len;               // 数据长度
  fdcan2_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  fdcan2_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 关闭速率切换
  fdcan2_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 传统的CAN模式
  fdcan2_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
  fdcan2_TxHeader.MessageMarker = 0;

  if (xSemaphoreTake(xCanTxMutex, pdMS_TO_TICKS(100)) == pdTRUE) // 获取锁
  {
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &fdcan2_TxHeader, msg); // 发送
    xSemaphoreGive(xCanTxMutex);                                    // 释放锁
    return 0;
  }
  return 1; // 获取锁超时
}

uint8_t FDCAN2_Receive_Msg(uint8_t *buf, uint16_t *Identifier)
{
  if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &fdcan2_RxHeader, buf) != HAL_OK)
    return 0; // 接收数据
  *Identifier = fdcan2_RxHeader.Identifier;
  return fdcan2_RxHeader.DataLength >> 16;
}

/* USER CODE END 1 */
