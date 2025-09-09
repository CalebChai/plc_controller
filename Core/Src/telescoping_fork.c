#include "telescoping_fork.h"
// 全局状态变量
ForkState currentState = FORK_STATE_INIT;
static uint8_t nodeId = 3; // CANopen节点ID
void telescopingForkInit(void)
{
    canopen_init(nodeId, fdcan2_TxHeader);
    canopen_sdo_write(nodeId, 0x1017, 0x00, 0x000003E8, 2); // 开启心跳报文
    osDelay(20);

    canopen_sdo_write(nodeId, 0x6099, 0x03, 0x00000000, 1); // 关闭自动回零
    osDelay(20);

    canopen_sdo_write(nodeId, 0x6060, 0x00, 0x00000001, 1); // 位置模式
    osDelay(20);

    canopen_sdo_write(nodeId, 0x6081, 0x00, 0x00085555, 4); // 梯形速度
    osDelay(20);

    currentState = FORK_STATE_IDLE;
}

void telescopingForkFSM(void)
{
    static uint32_t extendPosition = 1000; // 伸出目标位置（单位：脉冲）
    static uint32_t retractPosition = 0;   // 缩回目标位置

    switch (currentState)
    {
    case FORK_STATE_INIT:
        // 初始化处理
        telescopingForkInit();
        break;

    case FORK_STATE_IDLE:
        // 空闲状态等待命令
        // 此处可添加超时检测或安全监测
        break;

    case FORK_STATE_EXTENDING:
        // 货叉伸出动作
        canopen_sdo_write(nodeId, 0x607A, 0x00, extendPosition, 4); // 设置目标位置
        canopen_sdo_write(nodeId, 0x6040, 0x00, 0x000F, 2);         // 启动运动 (0x000F = 启动+立即执行)

        // 检查运动完成（实际应用中需通过PDO或状态字检测）
        // if (/* 位置到达检测 */)
        // {
        //     currentState = FORK_STATE_IDLE;
        // }
        break;

    case FORK_STATE_RETRACTING:
        // 货叉缩回动作
        canopen_sdo_write(nodeId, 0x607A, 0x00, retractPosition, 4);
        canopen_sdo_write(nodeId, 0x6040, 0x00, 0x000F, 2);

        // if (/* 位置到达检测 */)
        // {
        //     currentState = FORK_STATE_IDLE;
        // }
        break;

    case FORK_STATE_EMERGENCY:
        // 紧急停止处理
        canopen_sdo_write(nodeId, 0x6040, 0x00, 0x000B, 2); // 急停命令 (0x000B)
        currentState = FORK_STATE_IDLE;
        break;

    case FORK_STATE_ERROR:
        // 错误处理（如驱动器故障）
        // 此处可添加故障恢复逻辑
        break;
    }
}

// 状态转移接口函数
void forkExtend(void)
{
    if (currentState == FORK_STATE_IDLE)
    {
        currentState = FORK_STATE_EXTENDING;
    }
}

void forkRetract(void)
{
    if (currentState == FORK_STATE_IDLE)
    {
        currentState = FORK_STATE_RETRACTING;
    }
}

void forkEmergencyStop(void)
{
    // 可从任何状态触发急停
    currentState = FORK_STATE_EMERGENCY;
}

// // 示例：状态字检查函数（需根据实际驱动器手册实现）
// bool isMotionComplete(void)
// {
//     uint32_t statusWord = 0;
//     // 通过SDO读取状态字 (对象字典0x6041)
//     canopen_sdo_read(nodeId, 0x6041, 0x00, &statusWord, 2);

//     // 检查第10位 "Target reached" (典型值)
//     return (statusWord & (1 << 10)) != 0;
// }