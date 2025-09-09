#ifndef RS485_H
#define RS485_H
#include "main.h"
#include "usart.h"
#include <stdarg.h>
#include "stdint.h"
// 串口3为舵机rs485通信串口
// 串口10为超级电容rs485通信串口
// PD8 PE4为rs485控制引脚
#define RS485DIR_TX_SERVO HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);   // 定义我们的控制引脚为发送状态
#define RS485DIR_RX_SERVO HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET); // 定义我们的控制引脚为接收状态
#define RS485DIR_TX_POWER HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // 定义我们的控制引脚为发送状态
#define RS485DIR_RX_POWER HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); // 定义我们的控制引脚为接收状态
#define USART3_RXBUFFERSIZE 1                                                   // 每次接收1个数据进入一次中断
#define USART3_REC_LEN 200                                                      // 定义最大接收字节数 200
void RS485Init();
void RS485ServoTransmit(uint8_t *data, uint8_t len);
void RS485PowerTransmit(uint8_t *data, uint8_t len);
extern uint16_t USART3_RX_STA;
extern uint16_t USART10_RX_STA;
extern uint16_t USART2_RX_STA;
extern uint8_t RS485_rx_cnt;
extern uint8_t RS485_rx_buf[64];
#endif