#ifndef KINCO_SERVO_H
#define KINCO_SERVO_H
#include "fdcan.h"
#include "canopen.h"

/**
 * @brief  伺服电机控制模式枚举
 */
typedef enum
{
    CONTROL_MODE_POSITION = 0, // 位置模式
    CONTROL_MODE_VELOCITY,     // 速度模式
    CONTROL_MODE_TORQUE,       // 转矩模式
    CONTROL_MODE_HOMING        // 回零模式
} ServoControlMode;

/**
 * @brief  步科伺服电机结构体定义
 * @note   此结构体包含步科伺服电机的关键参数和状态信息
 */
typedef struct
{
    // 基本信息
    uint8_t id; // 电机ID（CAN/CANOpen或Modbus地址）
    // 机械参数
    float reductionRatio;         // 减速比
    float wheelDiameter;          // 轮径（单位：毫米）
    float encoderResolution;      // 编码器分辨率（脉冲/转）
    ServoControlMode controlMode; // 控制模式：位置/速度/转矩
    // 状态信息
    float actualPosition; // 实际位置（脉冲或角度）
    float actualSpeed;    // 实际速度（RPM）
    float actualTorque;   // 实际转矩（%）
    uint16_t statusWord;  // 状态字
    uint32_t errorCode;   // 错误代码

    // 回调函数指针
    void (*error_handler)(struct KincoServoMotor *motor);    // 错误处理回调
    void (*position_reached)(struct KincoServoMotor *motor); // 位置到达回调

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
