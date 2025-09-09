#ifndef KINCO_SERVO_H
#define KINCO_SERVO_H
#include "fdcan.h"
#include "canopen.h"

/**
 * @brief  �ŷ��������ģʽö��
 */
typedef enum
{
    CONTROL_MODE_POSITION = 0, // λ��ģʽ
    CONTROL_MODE_VELOCITY,     // �ٶ�ģʽ
    CONTROL_MODE_TORQUE,       // ת��ģʽ
    CONTROL_MODE_HOMING        // ����ģʽ
} ServoControlMode;

/**
 * @brief  �����ŷ�����ṹ�嶨��
 * @note   �˽ṹ����������ŷ�����Ĺؼ�������״̬��Ϣ
 */
typedef struct
{
    // ������Ϣ
    uint8_t id; // ���ID��CAN/CANOpen��Modbus��ַ��
    // ��е����
    float reductionRatio;         // ���ٱ�
    float wheelDiameter;          // �־�����λ�����ף�
    float encoderResolution;      // �������ֱ��ʣ�����/ת��
    ServoControlMode controlMode; // ����ģʽ��λ��/�ٶ�/ת��
    // ״̬��Ϣ
    float actualPosition; // ʵ��λ�ã������Ƕȣ�
    float actualSpeed;    // ʵ���ٶȣ�RPM��
    float actualTorque;   // ʵ��ת�أ�%��
    uint16_t statusWord;  // ״̬��
    uint32_t errorCode;   // �������

    // �ص�����ָ��
    void (*error_handler)(struct KincoServoMotor *motor);    // ������ص�
    void (*position_reached)(struct KincoServoMotor *motor); // λ�õ���ص�

} KincoServoMotor;

void kincoServoEnable(KincoServoMotor *motor);
void kincoServoDisable(KincoServoMotor *motor);
void kincoServoSetPosition(KincoServoMotor *motor, int32_t position, float velocity);
void kincoServoSetVelocity(KincoServoMotor *motor, float velocity);
void kincoServoHome(KincoServoMotor *motor);
void kincoServoReadStatus(KincoServoMotor *motor);
void kincoServoClearError(KincoServoMotor *motor);
float kincoServoGetActualPosition(KincoServoMotor *motor);
float kincoServoCalculatePulseToMM(KincoServoMotor *motor, int32_t pulses);
int32_t kincoServoCalculateMMToPulse(KincoServoMotor *motor, float mm);
#endif
