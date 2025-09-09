#ifndef SERVO_H
#define SERVO_H
#include "rs485.h"
extern UART_HandleTypeDef huart3;
extern uint8_t order_rx[64];

/* Э����س��� */
#define PACKET_HEADER_1 0xFF
#define PACKET_HEADER_2 0xFF
#define MAX_PACKET_LENGTH 32
#define PARAM_OFFSET 5

/* ����״̬��״̬ */
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

// ���ָ�����Ͷ���
typedef enum
{
    SERVO_PING = 0x01,  // ��ѯָ��
    SERVO_READ = 0x02,  // ��ȡ����
    SERVO_WRITE = 0x03, // д������
    SERVO_RESET = 0x06, // �ָ���������
} ServoInstruction;

// ����Ĵ�����ַ����
typedef enum
{
    SERVO_REG_FIRMWARE_VERSION_L = 0x03, // �̼��汾
    SERVO_REG_FIRMWARE_VERSION_H = 0x04,
    SERVO_REG_ID = 0x05,
    SERVO_REG_BAUD_RATE = 0x06,
    SERVO_REG_MAX_TORQUE_L = 0x10, // ���Ť��
    SERVO_REG_MAX_TORQUE_H = 0x11,
    SERVO_REG_POSITION_CALIBRATE_L = 0x21, // λ��У׼
    SERVO_REG_POSITION_CALIBRATE_H = 0x22,
    SERVO_REG_TORQUE_SWITCH = 0x28,   // Ť�ؿ���
    SERVO_REG_GOAL_POSITION_L = 0x2A, // Ŀ��λ��
    SERVO_REG_GOAL_POSITION_H = 0x2B,
    SERVO_REG_OPERATION_TIME_L = 0x2C, // ����ʱ��
    SERVO_REG_OPERATION_TIME_H = 0x2D,
    SERVO_REG_MOVING_SPEED_L = 0x2E, // �ƶ��ٶ�
    SERVO_REG_MOVING_SPEED_H = 0x2F,
    SERVO_REG_PRESENT_POSITION_L = 0x38, // ��ǰλ��
    SERVO_REG_PRESENT_POSITION_H = 0x39,
    SERVO_REG_STATUS = 0x41,    // ��ǰŤ��
    SERVO_REG_MAX_SPEED = 0x4E, // ����ٶ�
} ServoRegister;

/* ����������ݰ��ṹ */
#pragma pack(push, 1) // ����1�ֽڽ��ն���	ȷ���ṹ���С=����Ա�ֽ��ܺ�
typedef struct
{
    uint8_t header[2];  // 0xFF 0xFF
    uint8_t id;         // ���ID (1-250, 253-254)
    uint8_t length;     // ���ݳ��� (ID+���״̬+�������ֽ���)
    uint8_t status;     // ���״̬
    uint8_t params[16]; // ��������
    uint8_t checksum;   // У���
} ServoRxPacket;
#pragma pack(pop) // �ָ�֮ǰ�Ķ������� ����Ӱ����������

/* ����������ݰ��ṹ */
#pragma pack(push, 1) // ����1�ֽڽ��ն���	ȷ���ṹ���С=����Ա�ֽ��ܺ�
typedef struct
{
    uint8_t header[2];   // 0xFF 0xFF
    uint8_t id;          // ���ID (1-250, 253-254)
    uint8_t length;      // ���ݳ��� (ID+ָ������+�������ֽ���)
    uint8_t instruction; // ָ������
    uint8_t params[16];  // ��������
    uint8_t checksum;    // У���
} ServoTxPacket;
#pragma pack(pop) // �ָ�֮ǰ�Ķ������� ����Ӱ����������

/* ȫ�ֱ��� */
extern uint8_t rxBuffer[MAX_PACKET_LENGTH];
extern volatile UART_RxState rxState;
extern volatile uint8_t rxIndex;
extern volatile uint8_t rxParamCounter;
extern volatile uint8_t rxPacketLength;
extern volatile uint8_t rxPacketReady;

/* ����ԭ�� */
void ProcessServoPacket(ServoRxPacket *packet);
uint8_t CalculateChecksumRx(const uint8_t *data, uint8_t len);
uint8_t CalculateChecksumTx(const ServoTxPacket *packet);
ServoTxPacket createQueryIdPacket();
ServoTxPacket createSetPositionPacket(uint8_t servo_id, uint16_t position, uint16_t movingTime, uint16_t movingSpeed);
ServoTxPacket createPositionCalibratePacket(uint8_t servo_id, uint16_t positionCalibrate);
ServoTxPacket createReadPositionPacket(uint8_t servo_id);
void sendServoPacket(ServoTxPacket *packet);
#endif