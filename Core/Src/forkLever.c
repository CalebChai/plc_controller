#include "forkLever.h"
// ʹ��ǰ��ҪУ׼���ж����ȷ��λ����ͬ
void setForkLeverPos(uint16_t pos)
{
    ServoTxPacket packet = createSetPositionPacket(0xFD, pos, 0x0000, 0x0000); // ����id�����ж������Ŀ��λ�ã�������ٶȵ���
    sendServoPacket(&packet);
}