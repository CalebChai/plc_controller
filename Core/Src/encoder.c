#include "encoder.h"
Encoder encoder;
float RawCurrentSpeed, CurrentSpeed = 0;
int SpeedTail = 0, SpeedHead = 0, SpeedSize = 5;
float SpeedFilter[5] = {0};
// 换位
void swap(float *a, float *b)
{
    float temp = *a;
    *a = *b;
    *b = temp;
}

// 冒泡排序
void bubble_sort(float arr[], int len)
{
    int i, j;
    for (i = 0; i < len - 1; i++)
        for (j = 0; j < len - 1 - i; j++)
            if (arr[j] > arr[j + 1])
                swap(arr + j, arr + j + 1);
}

// 光电编码器初始化
void Encoder_Init()
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 开启编码器定时器
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);     // 开启编码器定时器更新中断,防溢出处理
    HAL_TIM_Base_Start_IT(&htim2);                  // 开启10ms定时器中断(后期改为软件定时器)
    __HAL_TIM_SET_COUNTER(&htim1, 10000);           // 编码器定时器初始值设定为10000
    encoder.loopNum = 0;                            // 溢出计数
}

void GetSpeed()
{
    RawCurrentSpeed = encoder.speed;
    if (((SpeedTail + 1) % SpeedSize) != SpeedHead && RawCurrentSpeed != 0) // 如果未满，则一直入队，直至满了
    {
        SpeedFilter[SpeedTail] = RawCurrentSpeed;
        SpeedTail = (SpeedTail + 1) % SpeedSize;
    }
    else
    {
        bubble_sort(SpeedFilter, SpeedSize); // 冒泡排序
        CurrentSpeed = SpeedFilter[2];
        SpeedHead = (SpeedHead + 1) % SpeedSize; // 出队
        SpeedFilter[SpeedTail] = RawCurrentSpeed;
        SpeedTail = (SpeedTail + 1) % SpeedSize; // 入队
    }
}