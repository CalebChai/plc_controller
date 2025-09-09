#ifndef __TELESCOPING_FORK_H__
#define __TELESCOPING_FORK_H__
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "canopen.h"
// 状态机状态定义
typedef enum
{
    FORK_STATE_INIT,       // 初始化状态
    FORK_STATE_IDLE,       // 空闲待命
    FORK_STATE_EXTENDING,  // 货叉伸出中
    FORK_STATE_RETRACTING, // 货叉缩回中
    FORK_STATE_EMERGENCY,  // 紧急停止
    FORK_STATE_ERROR       // 错误状态
} ForkState;

void telescopingForkInit(void);
extern ForkState currentState;
#endif