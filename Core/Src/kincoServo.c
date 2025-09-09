#include "kincoServo.h"
void kincoServoEnable(KincoServoMotor *motor)
{
    canopen_sdo_write(motor->id, 0x6040, 0x00, 0x0000000F, 2); // д������0F
}

void kincoServoDisable(KincoServoMotor *motor)
{
    canopen_sdo_write(motor->id, 0x6040, 0x00, 0x00000006, 2); // д������06
}

void kincoServoClearError(KincoServoMotor *motor)
{
    canopen_sdo_write(motor->id, 0x6040, 0x00, 0x00000086, 2); // д������86
}

void kincoServoHome(KincoServoMotor *motor)
{
    canopen_sdo_write(motor->id, 0x6040, 0x00, 0x0000000F, 2); // д������0F
    osDelay(10);
    canopen_sdo_write(motor->id, 0x6040, 0x00, 0x0000001F, 2); // д������1F
}
