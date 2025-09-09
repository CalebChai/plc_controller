#include "forkLever.h"
// 使用前需要校准所有舵机，确保位置相同
void setForkLeverPos(uint16_t pos)
{
    ServoTxPacket packet = createSetPositionPacket(0xFD, pos, 0x0000, 0x0000); // 超级id向所有舵机发送目标位置，以最快速度到达
    sendServoPacket(&packet);
}