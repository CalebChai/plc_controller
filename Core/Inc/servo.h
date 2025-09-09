#ifndef SERVO_H
#define SERVO_H
#include "rs485.h"
extern UART_HandleTypeDef huart3;
extern uint8_t order_rx[64];

/* 协议相关常量 */
#define PACKET_HEADER_1 0xFF
#define PACKET_HEADER_2 0xFF
#define MAX_PACKET_LENGTH 32
#define PARAM_OFFSET 5

/* 接收状态机状态 */
typedef enum
{
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_RECEIVE_ID,
    STATE_RECEIVE_LENGTH,
    STATE_RECEIVE_STATUS,
    STATE_RECEIVE_PARAMS,
    STATE_RECEIVE_CHECKSUM
} UART_RxState;

// 舵机指令类型定义
typedef enum
{
    SERVO_PING = 0x01,  // 查询指令
    SERVO_READ = 0x02,  // 读取数据
    SERVO_WRITE = 0x03, // 写入数据
    SERVO_RESET = 0x06, // 恢复出厂设置
} ServoInstruction;

// 舵机寄存器地址定义
typedef enum
{
    SERVO_REG_FIRMWARE_VERSION_L = 0x03, // 固件版本
    SERVO_REG_FIRMWARE_VERSION_H = 0x04,
    SERVO_REG_ID = 0x05,
    SERVO_REG_BAUD_RATE = 0x06,
    SERVO_REG_MAX_TORQUE_L = 0x10, // 最大扭矩
    SERVO_REG_MAX_TORQUE_H = 0x11,
    SERVO_REG_POSITION_CALIBRATE_L = 0x21, // 位置校准
    SERVO_REG_POSITION_CALIBRATE_H = 0x22,
    SERVO_REG_TORQUE_SWITCH = 0x28,   // 扭矩开关
    SERVO_REG_GOAL_POSITION_L = 0x2A, // 目标位置
    SERVO_REG_GOAL_POSITION_H = 0x2B,
    SERVO_REG_OPERATION_TIME_L = 0x2C, // 运行时间
    SERVO_REG_OPERATION_TIME_H = 0x2D,
    SERVO_REG_MOVING_SPEED_L = 0x2E, // 移动速度
    SERVO_REG_MOVING_SPEED_H = 0x2F,
    SERVO_REG_PRESENT_POSITION_L = 0x38, // 当前位置
    SERVO_REG_PRESENT_POSITION_H = 0x39,
    SERVO_REG_STATUS = 0x41,    // 当前扭矩
    SERVO_REG_MAX_SPEED = 0x4E, // 最大速度
} ServoRegister;

/* 舵机接收数据包结构 */
#pragma pack(push, 1) // 开启1字节紧凑对齐	确保结构体大小=各成员字节总和
typedef struct
{
    uint8_t header[2];  // 0xFF 0xFF
    uint8_t id;         // 舵机ID (1-250, 253-254)
    uint8_t length;     // 数据长度 (ID+舵机状态+参数的字节数)
    uint8_t status;     // 舵机状态
    uint8_t params[16]; // 参数数据
    uint8_t checksum;   // 校验和
} ServoRxPacket;
#pragma pack(pop) // 恢复之前的对齐设置 避免影响其他代码

/* 舵机发送数据包结构 */
#pragma pack(push, 1) // 开启1字节紧凑对齐	确保结构体大小=各成员字节总和
typedef struct
{
    uint8_t header[2];   // 0xFF 0xFF
    uint8_t id;          // 舵机ID (1-250, 253-254)
    uint8_t length;      // 数据长度 (ID+指令类型+参数的字节数)
    uint8_t instruction; // 指令类型
    uint8_t params[16];  // 参数数据
    uint8_t checksum;    // 校验和
} ServoTxPacket;
#pragma pack(pop) // 恢复之前的对齐设置 避免影响其他代码

/* 全局变量 */
extern uint8_t rxBuffer[MAX_PACKET_LENGTH];
extern volatile UART_RxState rxState;
extern volatile uint8_t rxIndex;
extern volatile uint8_t rxParamCounter;
extern volatile uint8_t rxPacketLength;
extern volatile uint8_t rxPacketReady;

/* 函数原型 */
void ProcessServoPacket(ServoRxPacket *packet);
uint8_t CalculateChecksumRx(const uint8_t *data, uint8_t len);
uint8_t CalculateChecksumTx(const ServoTxPacket *packet);
ServoTxPacket createQueryIdPacket();
ServoTxPacket createSetPositionPacket(uint8_t servo_id, uint16_t position, uint16_t movingTime, uint16_t movingSpeed);
ServoTxPacket createPositionCalibratePacket(uint8_t servo_id, uint16_t positionCalibrate);
ServoTxPacket createReadPositionPacket(uint8_t servo_id);
void sendServoPacket(ServoTxPacket *packet);
#endif