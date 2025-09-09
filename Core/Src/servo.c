#include "servo.h"
#include "rs485.h"

/* 全局变量 */
uint8_t rxBuffer[MAX_PACKET_LENGTH];
volatile UART_RxState rxState = STATE_WAIT_HEADER1;
volatile uint8_t rxIndex = 0;
volatile uint8_t rxParamCounter = 0; // 参数长度
volatile uint8_t rxPacketLength = 0; // 数据包长度
volatile uint8_t rxPacketReady = 0;

/* 计算校验和 */
uint8_t CalculateChecksumRx(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return ~sum; // 取反
}

// 计算校验和
uint8_t CalculateChecksumTx(const ServoTxPacket *packet)
{
    uint8_t sum = 0;

    // 计算ID + 长度 + 指令类型 的和
    sum += packet->id;
    sum += packet->length;
    sum += packet->instruction;

    // 计算参数的和
    // 参数长度 = length - 2 (减去ID和指令类型)
    uint8_t param_len = packet->length - 2;
    for (int i = 0; i < param_len; i++)
    {
        sum += packet->params[i];
    }

    return ~sum; // 取反得到校验和
}

/* 处理舵机数据包 */
void ProcessServoPacket(ServoRxPacket *packet)

{
    // 示例: 打印接收到的数据
    char msg[64];
    uint8_t paramLen = packet->length - 2; // 参数长度 = 总长度 - ID - 状态
    printf("ID:0x%02X, Status:0x%02X, Paramslen:%d\n", packet->id, packet->status, paramLen);
    // 打印参数 先发送低字节，再发送高字节
    for (uint8_t i = 0; i < paramLen; i++)
    {
        printf("0x%02X\n", packet->params[i]);
    }
    // 打印校验和
    printf("Checksum:0x%02X\n", packet->checksum);
    /*解析参数 */
    if (paramLen > 0)
    {
        // 检查参数格式 (带地址或不带地址)
        if (paramLen > 1 && packet->params[0] <= 0x7F)
        {
            // 可能是带地址的参数: 第一个字节是地址
            uint8_t address = packet->params[0];
            uint8_t *data = &packet->params[1];
            uint8_t dataLen = paramLen - 1;

            // 处理带地址的数据...
        }
        else
        {
            // 不带地址的参数
            // 直接处理参数数据...
        }
    }
}

// 创建舵机数据包
// id: 舵机ID
// instruction: 指令类型
// params: 参数数据
// param_len: 参数长度
void createServoPacket(ServoTxPacket *packet, uint8_t id,
                       uint8_t instruction,
                       const uint8_t *params, uint8_t param_len)
{
    // 设置包头
    packet->header[0] = 0xFF;
    packet->header[1] = 0xFF;

    // 设置舵机ID
    packet->id = id;

    // 设置指令类型
    packet->instruction = instruction;

    // 设置参数
    if (params != NULL && param_len > 0)
    {
        uint8_t actual_len = (param_len > 16) ? 16 : param_len;
        memcpy(packet->params, params, actual_len);

        // 设置数据长度 = ID(1) + 指令(1) + 参数长度
        packet->length = 2 + actual_len;
    }
    else
    {
        // 无参数指令
        packet->length = 2; // 只有ID和指令
    }

    // 计算并设置校验和
    packet->checksum = CalculateChecksumTx(packet);
}

// 创建查询ID数据包
ServoTxPacket createQueryIdPacket()
{
    ServoTxPacket packet;
    createServoPacket(&packet, 0xFD, SERVO_PING, NULL, 0);
    return packet;
}

// 创建舵机目标位置数据包
ServoTxPacket createSetPositionPacket(uint8_t servo_id, uint16_t position, uint16_t movingTime, uint16_t movingSpeed)
{
    // 参数格式: [寄存器地址, 数据低字节, 数据高字节]
    uint8_t params[7] = {
        SERVO_REG_GOAL_POSITION_L,     // 目标位置寄存器地址
        (uint8_t)(position & 0xFF),    // 位置低字节
        (uint8_t)(position >> 8),      // 位置高字节
        (uint8_t)(movingTime & 0xFF),  // 移动时间低字节
        (uint8_t)(movingTime >> 8),    // 移动时间高字节
        (uint8_t)(movingSpeed & 0xFF), // 移动速度低字节
        (uint8_t)(movingSpeed >> 8)    // 移动速度高字节
    };
    ServoTxPacket packet;
    createServoPacket(&packet, servo_id, SERVO_WRITE, params, 7);
    return packet;
}

// 创建位置校准数据包
ServoTxPacket createPositionCalibratePacket(uint8_t servo_id, uint16_t positionCalibrate)
{
    uint8_t params[3] = {
        SERVO_REG_POSITION_CALIBRATE_L,      // 目标位置寄存器地址
        (uint8_t)(positionCalibrate & 0xFF), // 位置低字节
        (uint8_t)(positionCalibrate >> 8),   // 位置高字节
    };
    ServoTxPacket packet;
    createServoPacket(&packet, servo_id, SERVO_WRITE, params, 3);
    return packet;
}

// 创建读取当前位置数据包
ServoTxPacket createReadPositionPacket(uint8_t servo_id)
{
    uint8_t params[2] = {
        SERVO_REG_PRESENT_POSITION_L, // 目标位置寄存器地址
        0x02};
    ServoTxPacket packet;
    createServoPacket(&packet, servo_id, SERVO_READ, params, 2);
    return packet;
}

// 发送舵机数据包
void sendServoPacket(ServoTxPacket *packet)
{

    // 计算数据包总长度 = 头部(2) + ID(1) + 长度(1) + 指令(1) + 参数(length-2) + 校验和(1)
    uint8_t totalLength = 6 + (packet->length - 2);
    packet->params[totalLength - 6] = packet->checksum;
    // 发送数据包
    RS485ServoTransmit((uint8_t *)packet, totalLength);
}