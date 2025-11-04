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
void AutoMoveTask(void *argument);
/* USER CODE END PTD */
CANopen_Status now = 0;
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t command_buffer[50];  // 用于存储接收到的命令
static uint8_t command_index = 0;  // 命令缓冲区索引
static uint32_t position = 0x00000000;
static uint32_t fork_position = 0x00000000;
// 每次递增 1000 (十进制，即 0x3E8)
#define POSITION_STEP_HEX   0x000003E8u
#define CMD_LINE_MAX 50  // 与 stm32h7xx_it.c 保持一致
int autoflag=1;
ServoTxPacket packet=0;
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
    .priority = (osPriority_t)osPriorityNormal1, // i2c 测试优先级最低
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

osThreadId_t AutoMoveTaskHandle;
const osThreadAttr_t AutoMoveTask_attributes = {
    .name = "AutoMoveTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
osMessageQueueId_t uart_rx_queue;
/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
static uint8_t output_status[10] = {0};  // 初始状态都为 0 (低电平)
static inline void forkExtend(int32_t delta)
{
    // 这里 position 是 uint32_t，如果不想让减法回绕，可以自行做饱和判断
    if (delta >= 0) {
        fork_position += (uint32_t)delta;
    } else {
        // 若不希望回绕，可改成：if (position < (uint32_t)(-delta)) position = 0; else position -= (uint32_t)(-delta);
        fork_position -= (uint32_t)(-delta);
    }

    // 写入新的目标位置并触发运动
    now=canopen_sdo_write(2, 0x6081, 0x00, 0x0014FFFF, 4); // 设定梽速度
    osDelay(20);
    now=canopen_sdo_write(2, 0x607A, 0x00, delta, 4); // 盠位置
    osDelay(20);
    now=canopen_sdo_write(2, 0x6040, 0x00, 0x004F, 2);   // 使能/准
    osDelay(20);
    now=canopen_sdo_write(2, 0x6040, 0x00, 0x005F, 2);   // 启动新位置运动
    osDelay(20);

    printf("move: position=0x%08lX (step=%ld)\n",
       (unsigned long)fork_position, (long)delta);

}
static inline void forkRetract(int32_t delta)
{
    // 这里 position 是 uint32_t，如果不想让减法回绕，可以自行做饱和判断
    if (delta >= 0) {
        fork_position += (uint32_t)delta;
    } else {
        // 若不希望回绕，可改成：if (position < (uint32_t)(-delta)) position = 0; else position -= (uint32_t)(-delta);
        fork_position -= (uint32_t)(-delta);
    }

    // 写入新的目标位置并触发运动
    now=canopen_sdo_write(2, 0x6081, 0x00, 0x0014FFFF, 4); // 设定梽速度
    osDelay(20);
    now=canopen_sdo_write(2, 0x607A, 0x00, delta, 4); // 盠位置
    osDelay(20);
    now=canopen_sdo_write(2, 0x6040, 0x00, 0x004F, 2);   // 使能/准
    osDelay(20);
    now=canopen_sdo_write(2, 0x6040, 0x00, 0x005F, 2);   // 启动新位置运动
    osDelay(20);

    printf("move_x: position=0 \n");
}
static inline void movewithSensor(int32_t delta)
{

}
static inline void do_move(int32_t delta)
{
  if(DigitalQuantity_Input(2)==0)// 
  {
    while(1)
    {
      printf("sensor!!!\n");
      osDelay(500);
      if(DigitalQuantity_Input(2)==1)// 
      {
        printf("ok!!!");
        break;
      }
    }
  }
  else
    // 这里 position 是 uint32_t，如果不想让减法回绕，可以自行做饱和判断
    {
      if (delta >= 0) {
        position += (uint32_t)delta;
    } else {
        // 若不希望回绕，可改成：if (position < (uint32_t)(-delta)) position = 0; else position -= (uint32_t)(-delta);
        position -= (uint32_t)(-delta);
    }

    // 写入新的目标位置并触发运动
    now=canopen_sdo_write(3, 0x6081, 0x00, 0x0014FFFF, 4); // 设定梽速度
    osDelay(20);    
    now=canopen_sdo_write(3, 0x607A, 0x00, delta, 4); // 盠位置
    osDelay(20);
    now=canopen_sdo_write(3, 0x6040, 0x00, 0x004F, 2);   // 使能/准
    osDelay(20);
    now=canopen_sdo_write(3, 0x6040, 0x00, 0x005F, 2);   // 启动新位置运动
    osDelay(20);

    printf("move_y: position=0x%08lX (step=%ld)\n",
           (unsigned long)position, (long)delta);
    }


    
}
static inline void sevroAction(int action)//1=90°，0=0°
{
  if(action)
  {
  packet = createSetPositionPacket(0x02, 0x0000, 0x0000, 0x0000);
  sendServoPacket(&packet);
  osDelay(20);
  packet = createSetPositionPacket(0x01, 0x0400, 0x0000, 0x0000);
  sendServoPacket(&packet);
  }
  else
  {
  packet = createSetPositionPacket(0x02, 0x0400, 0x0000, 0x0000);
  sendServoPacket(&packet);
  osDelay(20);
  packet = createSetPositionPacket(0x01, 0x0000, 0x0000, 0x0000);
  sendServoPacket(&packet);
  }
}

static void parse_command(const char *line)
{
    if (!line) { printf("无效命令: <null>\n"); return; }

    // 跳过前空白
    const char *p = line;
    while (*p == ' ' || *p == '\t') p++;

    // 取前三个字（足够判於
    char c0 = p[0];
    char c1 = p[1];
    char c2 = p[2];

    // ---- 方案A：两字符数字（端口电平）----
    // 允笸丘 '\0' / '\r' / 空格 / '\t'
    if (c0 >= '0' && c0 <= '9' && (c1 == '0' || c1 == '1') &&
        (c2 == '\0' || c2 == '\r' || c2 == ' ' || c2 == '\t'))
    {
        uint8_t port  = (uint8_t)(c0 - '0'); // 0..9
        uint8_t level = (uint8_t)(c1 - '0'); // 0/1
        if(port==0)
        {
          printf("port:%d/n",port);
          if(level)
          {
            ServoTxPacket packet = createSetPositionPacket(0x02, 0x0400, 0x0000, 0x0000);
            sendServoPacket(&packet);
          }
          else
          {
            ServoTxPacket packet = createSetPositionPacket(0x02, 0x0000, 0x0000, 0x0000);
            sendServoPacket(&packet);
          }
           
        }
        if (level)  DigitalQuantity_Output(port + 1, HIGH);
        else        DigitalQuantity_Output(port + 1, LOW);

        printf("控制输出端%u: %s\n", (unsigned)(port + 1), level ? "HIGH" : "LOW");
        return;
    }

    // ---- 方案B：单字符运动命令（W/S/A/D）---
    // 允许只有一个字符 + 终止/空白
    if ((c0 == 'W' || c0 == 'w' ||
         c0 == 'S' || c0 == 's' ||
         c0 == 'A' || c0 == 'a' ||
         c0 == 'D' || c0 == 'd') &&
        (c1 == '\0' || c1 == '\r' || c1 == ' ' || c1 == '\t'))
    {
        switch (c0) {
                case 'W': case 'w':        // 前进 1 step
                forkExtend((int32_t)POSITION_STEP_HEX);
                case 'D': case 'd':        // 右转 1 step
                do_move((int32_t)POSITION_STEP_HEX);
                break;

                case 'S': case 's':        // 后退 1 step
                forkRetract((int32_t)POSITION_STEP_HEX);
                case 'A': case 'a':        // 左转 1 step
                do_move(-(int32_t)POSITION_STEP_HEX);
                break;
        }
        return;
    }

    // 其它内容一律视为无效
    printf("无效命令: %s\n", line ? line : "<null>");
}

void parse_command_task(void *argument)
{
    uint8_t line[50];  // 队列里每条消恘行的必

    for (;;) {
        if (osMessageQueueGet(uart_rx_queue, line, NULL, osWaitForever) == osOK) {
            // 保证以'\0' 结尾（双重保险）
            line[sizeof(line)-1] = '\0';
            printf("Received command: %s\r\n", line);
            parse_command((const char*)line);
        }
    }
}


void MX_FREERTOS_Init(void)
{

    // 创建队列
    // 
    uart_rx_queue = osMessageQueueNew(8, CMD_LINE_MAX, NULL);

    // 
    osThreadNew(parse_command_task, NULL, NULL);

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
  // AutoMoveTaskHandle = osThreadNew(AutoMoveTask, NULL, &AutoMoveTask_attributes);
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
  // 
  xI2CMutex = xSemaphoreCreateMutex();
  xCanTxMutex = xSemaphoreCreateMutex();
  uint8_t data[4] = {1, 2, 3, 4};
  configASSERT(xI2CMutex != NULL); // FreeRTOSConfig.hconfigASSERT

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
  uint8_t input2_value_last = 0;
  // TCP_Echo_Init();
  while (1)
  {


    osDelay(2000);
    // canopen_sdo_write(2, 0x6040, 0x00, 0x000F, 2);   // 使能/准
		osDelay(5000);
    canopen_sdo_write(3, 0x6040, 0x00, 0x000F, 2);   // 使能/准
    // printf("aa");
    // 读取数据
    // tcp_read_data();
    // canopen_sdo_read(3, 0x1017, 0x00);

    // if (currentState == FORK_STATE_INIT)
    // {
    //   telescopingForkInit();
    //   printf("telescopingForkInit\n");
    // }
    // now = canopen_sdo_write(3, 0x6081, 0x00, 0x0014FFFF, 4); // 设定梽速度
    // else if (currentState == FORK_STATE_IDLE)
    // {
      // now = canopen_sdo_write(3, 0x607A, 0x00, 0x00007530, 4); // 设定盠位置
      // osDelay(20);
      // now = canopen_sdo_write(3, 0x6040, 0x00, 0x0000004F, 2); // 写控制字4F
      // osDelay(20);
      // now = canopen_sdo_write(3, 0x6040, 0x00, 0x0000005F, 2); // 写控制字5F
      // osDelay(2000);
    //   printf("telescopingForkFSM\n");
    // }
    now = canopen_sdo_write(2, 0x1017, 0x00, 0x000003E8, 2); // 开启心跳报文时间 1s
    osDelay(200);
    now = canopen_sdo_write(3, 0x1017, 0x00, 0x000003E8, 2); // 开启心跳报文时间 1s
    osDelay(200);
    now = canopen_sdo_write(3, 0x6099, 0x03, 0x00000000, 1); // 关闭自动回零
    osDelay(200);
    now = canopen_sdo_write(3, 0x6060, 0x00, 0x00000001, 1); // 设置工作模式
    osDelay(200);
    now = canopen_sdo_write(2, 0x6099, 0x03, 0x00000000, 1); // 关闭自动回零
    osDelay(200);
    now = canopen_sdo_write(2, 0x6060, 0x00, 0x00000001, 1); // 设置工作模式
    // osDelay(2000);
    // sevroAction(0);
    // osDelay(2000);
		// sevroAction(1);
    if(autoflag)
  {

    // TODO: fill in automatic movement logic
  printf("auto");
  osDelay(10000);
  canopen_sdo_write(3, 0x6040, 0x00, 0x000F, 2);   // 使能/准
  osDelay(200);
  canopen_sdo_write(2, 0x6040, 0x00, 0x000F, 2);   // 使能/准
  printf("start");


  sevroAction(1);//0°
  forkExtend(-150000);
  osDelay(5000);
  sevroAction(0);
  osDelay(1000);
  forkRetract(150000);
  osDelay(5000);

  do_move(-1000000);
  osDelay(15000);

  forkExtend(-150000);
  osDelay(5000);
  sevroAction(1);
  osDelay(1000);
  forkRetract(150000);
  osDelay(5000);
  
  do_move(1000000);
  osDelay(15000);

  do_move(-1000000);
  osDelay(15000);

  sevroAction(1);
  forkExtend(-150000);
  osDelay(5000);
  sevroAction(0);
  osDelay(1000);
  forkRetract(150000);
  osDelay(5000);

  do_move(1000000);
  osDelay(15000);

  forkExtend(-150000);
  osDelay(5000);
  sevroAction(1);
  osDelay(1000);
  forkRetract(150000);
  osDelay(5000);
  autoflag=0;

  }
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0)
    {
      printf("FIFO full\n");
    }

            input2_value = DigitalQuantity_Input(2); // 

      printf("%u",input2_value); 



    RS485ServoTransmit(data, 4);
    RS485PowerTransmit(data, 4);

      if (servoFlag) // 舵机位置校准以及测试
{
 // ServoTxPacket packet = createQueryIdPacket();
//  ServoTxPacket packet = createSetPositionPacket(0x02, 0x0000, 0x0000, 0x0000);
 // ServoTxPacket packet = createSetPositionPacket(0x01, 0x0000, 0x0000, 0x0001);
 // ServoTxPacket packet = createReadPositionPacket(0x01);
 // ServoTxPacket packet = createPositionCalibratePacket(0x01, 0xffc5);
 sendServoPacket(&packet);
     osDelay(5000);
 servoFlag = 0;
} 


  }
}


void ServoTask()
{
  while (1)
  {
    if (rxPacketReady)
    {
      rxPacketReady = 0;

      // 轍为数捌结构
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
  printf("-------tcp初化结束\n");
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
      // 发送测试数据
      tcp_send_data("Hello, Server!");
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // 每执1
  }
  /* USER CODE END StartTask02 */
}

void AutoMoveTask(void *argument)
{
  /* USER CODE BEGIN AutoMoveTask */
  if(autoflag)
  {
    // TODO: fill in automatic movement logic
  printf("auto");
  osDelay(10000);
  do_move(500000);
  osDelay(20000);
  sevroAction(1);
  forkExtend(-150000);
  osDelay(10000);
  sevroAction(0);
  osDelay(10000);
  forkRetract(0);
  osDelay(10000);
  do_move(0);
  osDelay(10000);
  forkExtend(-150000);
  osDelay(10000);
  sevroAction(1);
  osDelay(10000);
  forkRetract(0);
  osDelay(10000);
  autoflag=0;

  }
  /* USER CODE END AutoMoveTask */
}

/* USER CODE END Application */
