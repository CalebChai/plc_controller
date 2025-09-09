#include "rs485.h"
#include "servo.h"

void RS485Init()
{
    HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer, USART3_RXBUFFERSIZE); // 开启接收中断
}

void RS485ServoTransmit(uint8_t *data, uint8_t len)
{

    RS485DIR_TX_SERVO;                                     // 拉高PD8，更改RS485模式为发送
    HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 100); // 发送数据
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) != SET)
        ;              // 等待发送结束
    RS485DIR_RX_SERVO; // 拉低PD8，更改RS485模式为接收
}

void RS485PowerTransmit(uint8_t *data, uint8_t len)
{
    RS485DIR_TX_POWER;                                      // 拉高PE4，更改RS485模式为发送
    HAL_UART_Transmit(&huart10, (uint8_t *)data, len, 100); // 发送数据
    while (__HAL_UART_GET_FLAG(&huart10, UART_FLAG_TC) != SET)
        ;              // 等待发送结束
    RS485DIR_RX_POWER; // 拉低PE4，更改RS485模式为接收
}

uint8_t RS485_rx_cnt = 0;
uint8_t RS485_rx_buf[64] = {0}; // rs485接收缓冲
