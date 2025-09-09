#ifndef ENCODER_h
#define ENCODER_h
#include "main.h"
#include "tim.h"
#include "stdint.h"
#define RR 1u                                       // 电机减速比
#define RELOADVALUE __HAL_TIM_GetAutoreload(&htim1) // 获取自动装载值
#define COUNTERNUM __HAL_TIM_GetCounter(&htim1)     // 获取编码器定时器中的计数值
// 编码器结构体
typedef struct
{
    int32_t lastAngle;  // 上10ms转过的角度
    int32_t totalAngle; // 总的角度
    int16_t loopNum;    // 溢出次数计数值
    float speed;        // 编码器目前转速,单位为RPM
} Encoder;
extern Encoder encoder;
void Encoder_Init();
#endif