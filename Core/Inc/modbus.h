#ifndef __MODBUS_H__
#define __MODBUS_H__
#include "rs485.h"
#define MODBUS_SLAVE_ADDR 0x01
#define MODBUS_BAUDRATE 115200
#define MODBUS_TIMEOUT 35 // 3.5字符时间(ms)

// 寄存器映射
#define COIL_START_ADDR 0x0000
#define REG_HOLDING_START_ADDR 0x1000
#define REG_INPUT_START_ADDR 0x2000

// 寄存器数量
#define COIL_CNT 16
#define HOLDING_REG_CNT 32
#define INPUT_REG_CNT 16

#define MODBUS_USART huart10

// 通信状态
typedef struct
{
    uint8_t rx_buf[256];
    uint8_t tx_buf[256];
    uint16_t rx_len;
    uint16_t tx_len;
    uint8_t state;
} Modbus_HandleTypeDef;

extern Modbus_HandleTypeDef hmodbus;
void modbus_init(void);
void process_modbus_frame(void);
#endif
