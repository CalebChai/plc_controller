/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "digital_quantity.h"
#include "usart.h"
#include "rs485.h"
#include "canopen.h"
#include "servo.h"
#include "eeprom.h"
#include "lwip.h"
#include "tcpclient.h"
#include "tcpserver.h"
#include "telescoping_fork.h"
#include "forkLever.h"
#include "move.h"
#include "kincoServo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void DigitalOutputTestTask();
void ServoTask();
void lwipTask();
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t DigitalTestTaskHandle;
const osThreadAttr_t DigitalTestTask_attributes = {
    .name = "DigitalTestTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal1, // i2c测试优先级最高
};

osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
    .name = "ServoTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t lwipTaskHandle;
const osThreadAttr_t lwipTask_attributes = {
    .name = "lwipTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  DigitalTestTaskHandle = osThreadNew(DigitalOutputTestTask, NULL, &DigitalTestTask_attributes);
  // ServoTaskHandle = osThreadNew(ServoTask, NULL, &ServoTask_attributes);
  // lwipTaskHandle = osThreadNew(lwipTask, NULL, &lwipTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  // MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  printf("LwIP Init\n");

  /* Infinite loop */
  for (;;)
  {
    vTaskSuspend(defaultTaskHandle);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void DigitalOutputTestTask()
{
  // 创建互斥锁
  xI2CMutex = xSemaphoreCreateMutex();
  xCanTxMutex = xSemaphoreCreateMutex();
  uint8_t data[4] = {1, 2, 3, 4};
  configASSERT(xI2CMutex != NULL); // 在FreeRTOSConfig.h中启用configASSERT
  static uint8_t dataReceived[8] = {0};
  static uint8_t len = 0;
  static uint8_t lenReceived = 0;
  CANopen_Status now = 0;
  // osDelay(5000);
  /* Infinite loop */
  uint8_t servoFlag = 1;
  uint16_t startAddr = 0x2000;
  uint8_t writeData[256];
  uint8_t readData[256];
  // 2. 分页写入数据
  uint16_t bytesWritten = 0;
  uint8_t *ptr = writeData;
  uint16_t remaining = 256;
  uint16_t bytesRead = 0;
  uint8_t chunk = 0;
  static uint16_t errors = 0;
  uint8_t input2_value = 0;
  // TCP_Echo_Init();
  while (1)
  {
    printf("aa");
    // 读取数据
    // tcp_read_data();
    canopen_sdo_read(3, 0x1017, 0x00);

    if (currentState == FORK_STATE_INIT)
    {
      telescopingForkInit();
      printf("telescopingForkInit\n");
    }
    else if (currentState == FORK_STATE_IDLE)
    {
      now = canopen_sdo_write(3, 0x607A, 0x00, 0x00007530, 4); // 设定目标位置
      osDelay(20);
      now = canopen_sdo_write(3, 0x6040, 0x00, 0x0000004F, 2); // 写控制字4F
      osDelay(20);
      now = canopen_sdo_write(3, 0x6040, 0x00, 0x0000005F, 2); // 写控制字5F
      osDelay(20);
      printf("telescopingForkFSM\n");
    }

    now = canopen_sdo_write(3, 0x1017, 0x00, 0x000003E8, 2); // 开启心跳报文 时间为1s
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0)
    {
      printf("FIFO full\n");
    }
    FDCAN2_Send_Msg(dataReceived, 8, 0);
    printf("tx: %d\n", now);
    now = canopen_sdo_write(3, 0x6099, 0x03, 0x00000000, 1); // 关闭自动回零
    now = canopen_sdo_write(3, 0x6060, 0x00, 0x00000001, 1); // 设置工作模式
    now = canopen_sdo_write(3, 0x607A, 0x00, 0xFFFFD8F0, 4); // 设定目标位置
    now = canopen_sdo_write(3, 0x6081, 0x00, 0x00085555, 4); // 设定梯形速度
    // now = canopen_send_nmt(3, NMT_START_CMD);                // 管理节点进入operational状态开启PDO传输
    now = canopen_sdo_write(3, 0x6040, 0x00, 0x0000004F, 2); // 写控制字4F
    now = canopen_sdo_write(3, 0x6040, 0x00, 0x0000005F, 2); // 写控制字5F
    ServoTxPacket packet = createSetPositionPacket(0x02, 0x0400, 0x000, 0x0000);
    sendServoPacket(&packet);
    servoFlag=1;
    osDelay(5000);
    /* osDelay(3000);                                           // 步科伺服电机canopen测试
    now = canopen_sdo_write(3, 0x607A, 0x00, 0xFFFFD8F0, 4); // 设定目标位置
    osDelay(20);
    now = canopen_sdo_write(3, 0x6040, 0x00, 0x0000004F, 2); // 写控制字4F
    osDelay(20);
    now = canopen_sdo_write(3, 0x6040, 0x00, 0x0000005F, 2); // 写控制字5F
    osDelay(20); */

       input2_value = DigitalQuantity_Input(7); // 数字量输入输出测试
      //  DigitalQuantity_Output(1, LOW);
      //  DigitalQuantity_Output(2, LOW);
      //  DigitalQuantity_Output(3, HIGH);
      //  DigitalQuantity_Output(4, LOW);
      //  DigitalQuantity_Output(5, LOW);
       DigitalQuantity_Output(6, LOW);
       DigitalQuantity_Output(7, LOW);
       DigitalQuantity_Output(8, LOW);
       DigitalQuantity_Output(9, HIGH);
       DigitalQuantity_Output(10, LOW);
       osDelay(20); 

    RS485ServoTransmit(data, 4);
    RS485PowerTransmit(data, 4);

      if (servoFlag) // 舵机位置校准以及测试
{
 // ServoTxPacket packet = createQueryIdPacket();
 ServoTxPacket packet = createSetPositionPacket(0x02, 0x0000, 0x0000, 0x0000);
 // ServoTxPacket packet = createSetPositionPacket(0x01, 0x0000, 0x0000, 0x0001);
 // ServoTxPacket packet = createReadPositionPacket(0x01);
 // ServoTxPacket packet = createPositionCalibratePacket(0x01, 0xffc5);
 sendServoPacket(&packet);
     osDelay(5000);
 servoFlag = 0;
} 

    /*     // eepromWrite(0x0000, 0x55);//eeprom写入测试
        // uint8_t data = eepromRead(0x0000);
        // printf("data = 0x%x\n", data);

        printf("EEPROM Sequential Read/Write Test...\r\n");

        // 1. 生成测试数据
        for (int i = 0; i < 256; i++)
        {
          writeData[i] = i % 256; // 0-255循环
        }

        while (remaining > 0)
        {
          chunk = (remaining > PAGE_SIZE) ? PAGE_SIZE : remaining;
          if (eepromPageWrite(startAddr + bytesWritten, ptr, chunk))
          {
            bytesWritten += chunk;
            ptr += chunk;
            remaining -= chunk;
          }
          else
          {
            printf("Write failed at address 0x%04X\r\n", startAddr + bytesWritten);
            break;
          }
          osDelay(10); // 等待写周期完成
        }

        printf("Written %d bytes starting at 0x%04X\r\n", bytesWritten, startAddr);

        // 3. 顺序读取数据
        bytesRead = eepromSequentialRead(startAddr, readData, 256);

        if (bytesRead != 256)
        {
          printf("Read failed! Requested %d, got %d bytes.\r\n", 256, bytesRead);
          return;
        }

        // 4. 验证数据

        for (int i = 0; i < 256; i++)
        {
          if (readData[i] != writeData[i])
          {
            errors++;
            if (errors < 10)
            { // 最多打印10个错误
              printf("Error at 0x%04X: Wrote 0x%02X, Read 0x%02X\r\n",
                     startAddr + i, writeData[i], readData[i]);
            }
          }
        }

        if (errors == 0)
        {
          printf("Sequential read/write test PASSED!\r\n");
        }
        else
        {
          printf("Sequential read/write test FAILED! %d errors found.\r\n", errors);
        } */
  }
}

void ServoTask()
{
  while (1)
  {
    if (rxPacketReady)
    {
      rxPacketReady = 0;

      // 转换为数据包结构
      ServoRxPacket *packet = (ServoRxPacket *)rxBuffer;

      // 验证校验和
      uint8_t calcChecksum = CalculateChecksumRx(&rxBuffer[2], packet->length + 1);
      uint8_t receivedChecksum = rxBuffer[2 + packet->length + 1];
      packet->checksum = receivedChecksum;

      if (calcChecksum == receivedChecksum)
      {
        ProcessServoPacket(packet);
      }
      else
      {
        // 校验和错误处理
        // 例如: 重发请求或记录错误
      }
    }
    osDelay(10);
  }
}

void lwipTask()
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  osDelay(3000);
  tcp_client_init();
  printf("-------tcp初始化结束\n");
  for (;;)
  {
    if (!isConnected())
    {
      // 尝试重新连接
      reconnect_attempts++;
      if (reconnect_attempts <= MAX_RECONNECT_ATTEMPTS && !isConnected())
      {
        //  printf("Reconnecting... Attempt %d\n", reconnect_attempts);
        vTaskDelay(pdMS_TO_TICKS(1000));
        tcp_client_init();
      }
      else
      {
        //  printf("Max reconnect attempts reached. Giving up.\n");
        reconnect_attempts = 0;
      }
    }
    else
    {
      // 发�?�数�?
      tcp_send_data("Hello, Server!");
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒执行1
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE END Application */
