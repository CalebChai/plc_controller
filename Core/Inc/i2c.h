#ifndef I2C_H
#define I2C_H
#include "stdint.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
/* 软件I2C配置 - 根据实际硬件修改 */
#define SCL_PIN GPIO_PIN_12
#define SCL_PORT GPIOD
#define SDA_PIN GPIO_PIN_13
#define SDA_PORT GPIOD

/* I2C时序参数 (单位:微秒) */
#define I2C_DELAY_STD 5  // 标准模式延迟 (100kHz)
#define I2C_DELAY_FAST 2 // 快速模式延迟 (400kHz)

// 系统时钟频率（根据实际修改）
#define SYS_CLK_FREQ 550000000 // 550 MHz
// 每条循环的时钟周期数（需根据实际架构调整）
#define CYCLES_PER_LOOP 4

// I2C操作状态
typedef enum
{
    I2C_OK = 0,
    I2C_ERROR,
    I2C_TIMEOUT,
    I2C_BUSY
} I2C_Status;
// 互斥锁句柄声明
extern SemaphoreHandle_t xI2CMutex;

void Soft_I2C_SetSpeed(uint8_t mode);
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
uint8_t Soft_I2C_WriteByte(uint8_t byte);
uint8_t Soft_I2C_ReadByte(uint8_t ack);
uint8_t Soft_I2C_Write(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);
uint8_t Soft_I2C_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);

#endif