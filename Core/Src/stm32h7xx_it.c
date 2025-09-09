/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32h7xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rs485.h"
#include "stdint.h"
#include "fdcan.h"
#include "servo.h"
#include "modbus.h"
#include "cmsis_os.h"  // 包含 FreeRTOS 的头文件

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



// 在 stm32h7xx_it.c 中定义自己的变量
uint8_t stm32h7xx_command_buffer[50];  // 用于存储接收到的命令
static uint8_t stm32h7xx_command_index = 0;  // 命令缓冲区索引
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint16_t USART3_RX_STA = 0;
uint16_t USART10_RX_STA = 0;
uint16_t USART2_RX_STA = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern FDCAN_HandleTypeDef hfdcan2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart10;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */
extern uint8_t command_buffer[50];  // 声明 command_buffer
extern uint8_t command_index;  // 声明 command_index
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles FDCAN2 interrupt 0.
 */
void FDCAN2_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 0 */

  /* USER CODE END FDCAN2_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 1 */
  // printf("%X\n", data[0]);
  /* USER CODE END FDCAN2_IT0_IRQn 1 */
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
 * @brief This function handles USART3 global interrupt.
 */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
 */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
 * @brief This function handles Ethernet global interrupt.
 */
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/**
 * @brief This function handles USART10 global interrupt.
 */
void USART10_IRQHandler(void)
{
  /* USER CODE BEGIN USART10_IRQn 0 */

  /* USER CODE END USART10_IRQn 0 */
  HAL_UART_IRQHandler(&huart10);
  /* USER CODE BEGIN USART10_IRQn 1 */

  /* USER CODE END USART10_IRQn 1 */
}
extern osMessageQueueId_t uart_rx_queue;
/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t rxByte;

  // 串口3为舵机通信串口
  if (huart->Instance == USART3)
  { // 根据实际串口修改
    /* 读取接收到的字节 */
    rxByte = rxBuffer[0]; // 串口接收到的最新的字节会放在rxBuffer[0]中

    switch (rxState)
    {
    case STATE_WAIT_HEADER1:
      if (rxByte == PACKET_HEADER_1)
      {
        rxBuffer[rxIndex++] = rxByte;
        rxState = STATE_WAIT_HEADER2;
      }
      break;

    case STATE_WAIT_HEADER2:
      if (rxByte == PACKET_HEADER_2)
      {
        rxBuffer[rxIndex++] = rxByte;
        rxState = STATE_RECEIVE_ID;
      }
      else
      {
        // 如果不是0xFF，重新开始接收
        rxIndex = 0;
        rxState = STATE_WAIT_HEADER1;
      }
      break;

    case STATE_RECEIVE_ID:
      rxBuffer[rxIndex++] = rxByte;
      rxState = STATE_RECEIVE_LENGTH;
      break;

    case STATE_RECEIVE_LENGTH:
      rxBuffer[rxIndex++] = rxByte;
      rxPacketLength = rxByte + 4; // 总包长 = 长度字段 + 4
      rxState = STATE_RECEIVE_STATUS;
      break;

    case STATE_RECEIVE_STATUS:
      rxBuffer[rxIndex++] = rxByte;
      // 计算参数长度 = 长度字段 - 2 (ID和状态)
      rxParamCounter = rxBuffer[3] - 2;

      if (rxParamCounter > 0)
      {
        rxState = STATE_RECEIVE_PARAMS;
      }
      else
      {
        rxState = STATE_RECEIVE_CHECKSUM;
      }
      break;

    case STATE_RECEIVE_PARAMS:
      rxBuffer[rxIndex++] = rxByte;
      if (--rxParamCounter == 0)
      {
        rxState = STATE_RECEIVE_CHECKSUM;
      }
      break;

    case STATE_RECEIVE_CHECKSUM:
      rxBuffer[rxIndex] = rxByte;

      // 数据包接收完成
      rxPacketReady = 1;

      // 重置状态机
      rxIndex = 0;
      rxState = STATE_WAIT_HEADER1;
      break;

    default:
      rxIndex = 0;
      rxState = STATE_WAIT_HEADER1;
      break;
    }

    // 重新启动接收
    HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer, 1);
  }

if (huart->Instance == USART2)
    {
        uint8_t received_data = USART2_aRxBuffer[0];  // 接收到的数据

        // 如果接收到换行符，表示命令结束
        if (received_data == '\n' || received_data == '\r')  // 换行或回车
        {
            stm32h7xx_command_buffer[stm32h7xx_command_index] = '\0';  // 添加结束符
            osMessageQueuePut(uart_rx_queue, stm32h7xx_command_buffer, 0, 0);  // 将完整命令放入队列
            stm32h7xx_command_index = 0;  // 重置命令缓冲区索引
        }
        else
        {
            if (stm32h7xx_command_index < 50 - 1)  // 确保缓冲区不会溢出
            {
                stm32h7xx_command_buffer[stm32h7xx_command_index++] = received_data;  // 存储接收到的数据
            }
        }

        // 重新启动接收中断，准备接收下一个字节
        HAL_UART_Receive_IT(&huart2, USART2_aRxBuffer, 1);
    }
  if (huart->Instance == USART10)
  {
    // 重启超时定时器
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_TIM_Base_Start_IT(&htim6);

    // 存储接收数据
    uint8_t byte;
    byte = USART10_aRxBuffer[0];
    hmodbus.rx_buf[hmodbus.rx_len++] = byte;
    // 重启接收中断
    HAL_UART_Receive_IT(&MODBUS_USART, (uint8_t *)USART10_aRxBuffer, 1);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  uint8_t i = 0;
  uint8_t rxdata[8] = {0};
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan2_RxHeader, rxdata);
  printf("id:%#x\r\n", fdcan2_RxHeader.Identifier);
  printf("len:%d\r\n", fdcan2_RxHeader.DataLength);
  printf("rxdata:0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x\r\n", rxdata[0], rxdata[1], rxdata[2], rxdata[3], rxdata[4], rxdata[5], rxdata[6], rxdata[7]);
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* USER CODE END 1 */
