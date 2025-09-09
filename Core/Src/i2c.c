#include "i2c.h"
/* 当前工作模式 */
static uint8_t i2c_delay = I2C_DELAY_STD;
// 互斥锁实例
SemaphoreHandle_t xI2CMutex = NULL;

/* 微秒级延迟函数 */
static void I2C_Delay(void)
{
    // 计算所需的总时钟周期数
    uint32_t total_cycles = (uint32_t)(i2c_delay * 1e-6 * SYS_CLK_FREQ);

    // 计算循环次数
    uint32_t loop_count = total_cycles / CYCLES_PER_LOOP;

    // 确保最小循环次数
    if (loop_count < 1)
        loop_count = 1;

    // 执行延迟循环
    while (loop_count--)
    {
        __NOP();
    }
}

// 获取I2C总线访问权（带超时）
static I2C_Status I2C_AcquireBus(TickType_t timeout)
{
    if (xSemaphoreTake(xI2CMutex, timeout) != pdTRUE)
    {
        return I2C_BUSY;
    }
    return I2C_OK;
}

// 释放I2C总线
static void I2C_ReleaseBus(void)
{
    xSemaphoreGive(xI2CMutex);
}

/* 设置I2C速度模式 */
void Soft_I2C_SetSpeed(uint8_t mode)
{
    i2c_delay = (mode == 0) ? I2C_DELAY_STD : I2C_DELAY_FAST;
}

/* 产生I2C起始条件 */
void Soft_I2C_Start(void)
{
    // SDA高->低，同时SCL保持高
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();

    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();

    // 拉低SCL完成起始条件
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
}

/* 产生I2C停止条件 */
void Soft_I2C_Stop(void)
{
    // SDA低->高，同时SCL保持高
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();

    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    I2C_Delay();
}

/* 发送一个字节并检查ACK */
uint8_t Soft_I2C_WriteByte(uint8_t data)
{
    I2C_Status status = I2C_AcquireBus(pdMS_TO_TICKS(50));
    if (status != I2C_OK)
        return status;

    uint8_t ack;

    // 发送8位数据 (MSB first)
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1;

        // 产生时钟脉冲
        HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
        I2C_Delay();
        HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
        I2C_Delay();
    }

    // 读取ACK (第9个时钟周期)
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET); // 释放SDA
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();

    ack = HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN); // 0: ACK, 1: NACK

    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    I2C_ReleaseBus();
    return (ack == 0) ? 0 : 1; // 返回0表示收到ACK
}

/* 读取一个字节并发送ACK/NACK */
uint8_t Soft_I2C_ReadByte(uint8_t ack)
{
    I2C_Status status = I2C_AcquireBus(pdMS_TO_TICKS(50));
    if (status != I2C_OK)
        return status;
    uint8_t data = 0;

    // 释放SDA线
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);

    // 读取8位数据 (MSB first)
    for (uint8_t i = 0; i < 8; i++)
    {
        data <<= 1;

        HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
        I2C_Delay();

        if (HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN))
        {
            data |= 0x01;
        }

        HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
        I2C_Delay();
    }

    // 发送ACK/NACK (第9个时钟周期)
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, ack ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();

    // 恢复SDA为高电平
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    I2C_ReleaseBus();
    return data;
}

/* 向指定设备写入数据 */
uint8_t Soft_I2C_Write(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    uint8_t result;

    Soft_I2C_Start();

    // 发送设备地址 + 写位
    if (Soft_I2C_WriteByte(devAddr << 1 | 0x00))
    {
        Soft_I2C_Stop();
        return 1; // 设备无响应
    }

    // 发送寄存器地址
    if (Soft_I2C_WriteByte(regAddr))
    {
        Soft_I2C_Stop();
        return 2; // 寄存器地址无响应
    }

    // 发送数据
    for (uint8_t i = 0; i < len; i++)
    {
        if (Soft_I2C_WriteByte(data[i]))
        {
            Soft_I2C_Stop();
            return 3; // 数据写入失败
        }
    }

    Soft_I2C_Stop();
    return 0; // 成功
}

/* 从指定设备读取数据 */
uint8_t Soft_I2C_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    Soft_I2C_Start();

    // 发送设备地址 + 写位
    if (Soft_I2C_WriteByte(devAddr << 1 | 0x00))
    {
        Soft_I2C_Stop();
        return 1; // 设备无响应
    }

    // 发送寄存器地址
    if (Soft_I2C_WriteByte(regAddr))
    {
        Soft_I2C_Stop();
        return 2; // 寄存器地址无响应
    }

    // 重新启动以切换到读模式
    Soft_I2C_Start();

    // 发送设备地址 + 读位
    if (Soft_I2C_WriteByte(devAddr << 1 | 0x01))
    {
        Soft_I2C_Stop();
        return 3; // 设备读模式无响应
    }

    // 读取数据
    for (uint8_t i = 0; i < len; i++)
    {
        // 最后一个字节发送NACK，其它发送ACK
        data[i] = Soft_I2C_ReadByte((i == (len - 1)) ? 1 : 0);
    }

    Soft_I2C_Stop();
    return 0; // 成功
}

/* 扫描I2C总线上的设备 */
void Soft_I2C_Scan(void)
{
    printf("I2C Scan:\r\n");

    for (uint8_t addr = 1; addr < 128; addr++)
    {
        Soft_I2C_Start();
        uint8_t ack = Soft_I2C_WriteByte(addr << 1); // 尝试写操作
        Soft_I2C_Stop();

        if (!ack)
        {
            printf("Device found at 0x%02X\r\n", addr);
        }

        osDelay(1);
    }

    printf("Scan complete.\r\n");
}