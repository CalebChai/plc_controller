#include "eeprom.h"

uint8_t eepromWrite(uint16_t memAddr, uint8_t data)
{
    Soft_I2C_Start();
    // 发送器件地址 + 写标志
    if (Soft_I2C_WriteByte(EEPROM_I2C_ADDR << 1))
    {
        // 无ACK，设备未响应
        Soft_I2C_Stop();
        return 0;
    }

    // 发送存储器地址高字节
    if (Soft_I2C_WriteByte((memAddr >> 8) & 0xFF))
    {
        Soft_I2C_Stop();
        return 0;
    }

    // 发送存储器地址低字节
    if (Soft_I2C_WriteByte(memAddr & 0xFF))
    {
        Soft_I2C_Stop();
        return 0;
    }

    // 发送数据字节
    if (Soft_I2C_WriteByte(data))
    {
        Soft_I2C_Stop();
        return 0;
    }

    // 发送停止条件
    Soft_I2C_Stop();

    // 等待内部写周期完成
    osDelay(5);

    return 1; // 写入成功
}

uint8_t eepromRead(uint16_t memAddr)
{
    uint8_t data = 0;
    Soft_I2C_Start();
    // 发送器件地址 + 写标志 (设置读地址)
    if (Soft_I2C_WriteByte(EEPROM_I2C_ADDR << 1))
    {
        // 无ACK，设备未响应
        Soft_I2C_Stop();
        return 0xFF; // 返回错误值
    }

    // 发送存储器地址高字节
    if (Soft_I2C_WriteByte((memAddr >> 8) & 0xFF))
    {
        Soft_I2C_Stop();
        return 0xFF;
    }

    // 发送存储器地址低字节
    if (Soft_I2C_WriteByte(memAddr & 0xFF))
    {
        Soft_I2C_Stop();
        return 0xFF;
    }

    // 发送重复起始条件
    Soft_I2C_Start();

    // 发送器件地址 + 读标志
    if (Soft_I2C_WriteByte((EEPROM_I2C_ADDR << 1) | 0x01))
    {
        Soft_I2C_Stop();
        return 0xFF;
    }

    // 读取数据字节 (发送NACK表示只读一个字节)
    data = Soft_I2C_ReadByte(1); // 1表示发送NACK

    // 发送停止条件
    Soft_I2C_Stop();

    return data;
}

/**
 * @brief  EEPROM页写函数（最多64字节）
 * @param  memAddr: 起始存储器地址
 * @param  data: 要写入的数据数组
 * @param  len: 要写入的数据长度（1-64字节）
 * @retval 写入是否成功: 1=成功, 0=失败
 */
uint8_t eepromPageWrite(uint16_t memAddr, uint8_t *data, uint8_t len)
{
    // 检查页边界限制
    uint8_t pageOffset = memAddr % PAGE_SIZE;
    uint8_t remainingInPage = PAGE_SIZE - pageOffset;

    if (len == 0)
        return 0; // 无效长度
    if (len > remainingInPage)
    {
        // 自动截断到页边界
        len = remainingInPage;
        printf("Warning: Page boundary crossed. Truncating to %d bytes.\r\n", len);
    }
    if (len > PAGE_SIZE)
        len = PAGE_SIZE; // 安全限制

    // 发送起始条件
    Soft_I2C_Start();

    // 发送器件地址 + 写标志
    if (Soft_I2C_WriteByte(EEPROM_I2C_ADDR << 1))
    {
        Soft_I2C_Stop();
        return 0; // 设备未响应
    }

    // 发送存储器地址高字节
    if (Soft_I2C_WriteByte((memAddr >> 8) & 0xFF))
    {
        Soft_I2C_Stop();
        return 0;
    }

    // 发送存储器地址低字节
    if (Soft_I2C_WriteByte(memAddr & 0xFF))
    {
        Soft_I2C_Stop();
        return 0;
    }

    // 发送数据字节（最多64字节）
    for (uint8_t i = 0; i < len; i++)
    {
        if (Soft_I2C_WriteByte(data[i]))
        {
            // 未收到ACK，写入失败
            Soft_I2C_Stop();
            return 0;
        }
    }

    // 发送停止条件
    Soft_I2C_Stop();

    // 等待内部写周期完成
    osDelay(10);

    return 1; // 写入成功
}

/**
 * @brief  EEPROM顺序地址读取函数
 * @param  startAddr: 起始存储器地址
 * @param  data: 数据接收缓冲区
 * @param  len: 要读取的数据长度
 * @retval 实际读取的字节数
 */
uint16_t eepromSequentialRead(uint16_t startAddr, uint8_t *data, uint16_t len)
{
    if (len == 0)
        return 0;

    // 初始化地址计数器（通过随机地址读操作）
    // 发送起始条件
    Soft_I2C_Start();

    // 发送器件地址 + 写标志
    if (Soft_I2C_WriteByte(EEPROM_I2C_ADDR << 1))
    {
        Soft_I2C_Stop();
        return 0; // 设备未响应
    }

    // 发送存储器地址高字节
    if (Soft_I2C_WriteByte((startAddr >> 8) & 0xFF))
    {
        Soft_I2C_Stop();
        return 0;
    }

    // 发送存储器地址低字节
    if (Soft_I2C_WriteByte(startAddr & 0xFF))
    {
        Soft_I2C_Stop();
        return 0;
    }

    // 发送重复起始条件
    Soft_I2C_Start();

    // 发送器件地址 + 读标志
    if (Soft_I2C_WriteByte((EEPROM_I2C_ADDR << 1) | 0x01))
    {
        Soft_I2C_Stop();
        return 0;
    }

    // 顺序读取数据
    uint16_t bytesRead = 0;
    while (len > 0)
    {
        // 读取数据字节
        uint8_t ack = (len > 1) ? 0 : 1; // 最后一个字节发送NACK
        *data = Soft_I2C_ReadByte(ack);
        data++;
        bytesRead++;
        len--;

        // 地址计数器自动递增（超过0x7FFF会回滚到0x0000）
    }

    // 发送停止条件
    Soft_I2C_Stop();

    return bytesRead;
}