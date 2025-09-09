#include "modbus.h"

// 寄存器存储区
uint8_t coil_reg[COIL_CNT] = {0};
uint16_t holding_reg[HOLDING_REG_CNT] = {0};
uint16_t input_reg[INPUT_REG_CNT] = {0};
Modbus_HandleTypeDef hmodbus;
static uint16_t modbus_crc16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF; // CRC初始值

    while (len--) // 对数据每个字节进行处理
    {
        crc ^= *data++;                 // 异或当前字节，指针后移
        for (uint8_t i = 0; i < 8; i++) // 对当前字节的每个位进行处理
        {
            if (crc & 0x0001) // 检查最低位是否为1
            {
                crc = (crc >> 1) ^ 0xA001; // CRC校验多项式反转形式
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 构建异常响应
static void build_exception(uint8_t func, uint8_t code)
{
    hmodbus.tx_buf[0] = MODBUS_SLAVE_ADDR;
    hmodbus.tx_buf[1] = func | 0x80; // 设置异常标志
    hmodbus.tx_buf[2] = code;

    uint16_t crc = modbus_crc16(hmodbus.tx_buf, 3);
    hmodbus.tx_buf[3] = crc & 0xFF;
    hmodbus.tx_buf[4] = crc >> 8;
    hmodbus.tx_len = 5;
}

// 处理保持寄存器读取
static void handle_read_holding_regs(void)
{
    uint16_t start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
    uint16_t reg_count = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

    // 地址校验
    if (start_addr < REG_HOLDING_START_ADDR ||
        (start_addr + reg_count) > (REG_HOLDING_START_ADDR + HOLDING_REG_CNT))
    {
        build_exception(0x03, 0x02); // 非法数据地址
        return;
    }

    // 构建响应
    uint16_t index = 0;
    hmodbus.tx_buf[index++] = MODBUS_SLAVE_ADDR;
    hmodbus.tx_buf[index++] = 0x03;
    hmodbus.tx_buf[index++] = reg_count * 2;

    for (uint16_t i = 0; i < reg_count; i++)
    {
        uint16_t reg_val = holding_reg[start_addr - REG_HOLDING_START_ADDR + i];
        hmodbus.tx_buf[index++] = reg_val >> 8;
        hmodbus.tx_buf[index++] = reg_val & 0xFF;
    }

    // 添加CRC
    uint16_t crc = modbus_crc16(hmodbus.tx_buf, index);
    hmodbus.tx_buf[index++] = crc & 0xFF;
    hmodbus.tx_buf[index++] = crc >> 8;
    hmodbus.tx_len = index;
}

// 处理读取线圈状态 (0x01)
static void handle_read_coils(void)
{
    uint16_t start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
    uint16_t coil_count = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

    // 地址校验
    if (start_addr < COIL_START_ADDR ||
        (start_addr + coil_count) > (COIL_START_ADDR + COIL_CNT))
    {
        build_exception(0x01, 0x02); // 非法数据地址
        return;
    }

    // 计算所需字节数
    uint8_t byte_count = (coil_count + 7) / 8;
    uint16_t index = 0;

    // 构建响应头
    hmodbus.tx_buf[index++] = MODBUS_SLAVE_ADDR;
    hmodbus.tx_buf[index++] = 0x01;
    hmodbus.tx_buf[index++] = byte_count;

    // 打包线圈状态到字节
    uint16_t coil_index = start_addr - COIL_START_ADDR;
    for (uint8_t i = 0; i < byte_count; i++)
    {
        uint8_t coil_byte = 0;
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((i * 8 + j) < coil_count)
            {
                if (coil_reg[coil_index + i * 8 + j])
                {
                    coil_byte |= (1 << j);
                }
            }
        }
        hmodbus.tx_buf[index++] = coil_byte;
    }

    // 添加CRC
    uint16_t crc = modbus_crc16(hmodbus.tx_buf, index);
    hmodbus.tx_buf[index++] = crc & 0xFF;
    hmodbus.tx_buf[index++] = crc >> 8;
    hmodbus.tx_len = index;
}

// 处理写单个线圈 (0x05)
static void handle_write_coil(void)
{
    uint16_t coil_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
    uint16_t coil_value = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

    // 地址校验
    if (coil_addr < COIL_START_ADDR || coil_addr >= (COIL_START_ADDR + COIL_CNT))
    {
        build_exception(0x05, 0x02); // 非法数据地址
        return;
    }

    // 值校验 (必须为0xFF00或0x0000)
    if (coil_value != 0xFF00 && coil_value != 0x0000)
    {
        build_exception(0x05, 0x03); // 非法数据值
        return;
    }

    // 更新线圈状态
    uint16_t index = coil_addr - COIL_START_ADDR;
    coil_reg[index] = (coil_value == 0xFF00) ? 1 : 0;

    // 构建响应 (回显原始请求)
    uint16_t resp_index = 0;
    hmodbus.tx_buf[resp_index++] = MODBUS_SLAVE_ADDR;
    hmodbus.tx_buf[resp_index++] = 0x05;

    // 复制地址和值
    for (uint8_t i = 2; i < 6; i++)
    {
        hmodbus.tx_buf[resp_index++] = hmodbus.rx_buf[i];
    }

    // 添加CRC
    uint16_t crc = modbus_crc16(hmodbus.tx_buf, resp_index);
    hmodbus.tx_buf[resp_index++] = crc & 0xFF;
    hmodbus.tx_buf[resp_index++] = crc >> 8;
    hmodbus.tx_len = resp_index;
}

// 处理写单个寄存器 (0x06)
static void handle_write_reg(void)
{
    uint16_t reg_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
    uint16_t reg_value = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

    // 地址校验
    if (reg_addr < REG_HOLDING_START_ADDR ||
        reg_addr >= (REG_HOLDING_START_ADDR + HOLDING_REG_CNT))
    {
        build_exception(0x06, 0x02); // 非法数据地址
        return;
    }

    // 更新寄存器值
    uint16_t index = reg_addr - REG_HOLDING_START_ADDR;
    holding_reg[index] = reg_value;

    // 构建响应 (回显原始请求)
    uint16_t resp_index = 0;
    hmodbus.tx_buf[resp_index++] = MODBUS_SLAVE_ADDR;
    hmodbus.tx_buf[resp_index++] = 0x06;

    // 复制地址和值
    for (uint8_t i = 2; i < 6; i++)
    {
        hmodbus.tx_buf[resp_index++] = hmodbus.rx_buf[i];
    }

    // 添加CRC
    uint16_t crc = modbus_crc16(hmodbus.tx_buf, resp_index);
    hmodbus.tx_buf[resp_index++] = crc & 0xFF;
    hmodbus.tx_buf[resp_index++] = crc >> 8;
    hmodbus.tx_len = resp_index;
}

// 处理写多个寄存器 (0x10)
static void handle_write_multiple_regs(void)
{
    uint16_t start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
    uint16_t reg_count = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];
    uint8_t byte_count = hmodbus.rx_buf[6];

    // 地址校验
    if (start_addr < REG_HOLDING_START_ADDR ||
        (start_addr + reg_count) > (REG_HOLDING_START_ADDR + HOLDING_REG_CNT))
    {
        build_exception(0x10, 0x02); // 非法数据地址
        return;
    }

    // 数据长度校验
    if (byte_count != reg_count * 2)
    {
        build_exception(0x10, 0x03); // 非法数据值
        return;
    }

    // 更新寄存器值
    uint16_t data_index = 7;
    uint16_t reg_index = start_addr - REG_HOLDING_START_ADDR;

    for (uint16_t i = 0; i < reg_count; i++)
    {
        holding_reg[reg_index + i] = (hmodbus.rx_buf[data_index] << 8) | hmodbus.rx_buf[data_index + 1];
        data_index += 2;
    }

    // 构建响应
    uint16_t resp_index = 0;
    hmodbus.tx_buf[resp_index++] = MODBUS_SLAVE_ADDR;
    hmodbus.tx_buf[resp_index++] = 0x10;

    // 复制地址和寄存器数量
    hmodbus.tx_buf[resp_index++] = hmodbus.rx_buf[2]; // 起始地址高字节
    hmodbus.tx_buf[resp_index++] = hmodbus.rx_buf[3]; // 起始地址低字节
    hmodbus.tx_buf[resp_index++] = hmodbus.rx_buf[4]; // 寄存器数量高字节
    hmodbus.tx_buf[resp_index++] = hmodbus.rx_buf[5]; // 寄存器数量低字节

    // 添加CRC
    uint16_t crc = modbus_crc16(hmodbus.tx_buf, resp_index);
    hmodbus.tx_buf[resp_index++] = crc & 0xFF;
    hmodbus.tx_buf[resp_index++] = crc >> 8;
    hmodbus.tx_len = resp_index;
}

// 处理Modbus帧
void process_modbus_frame(void)
{
    // 校验长度
    if (hmodbus.rx_len < 4)
        return; // 最小帧长度

    // 校验CRC
    uint16_t crc_calc = modbus_crc16(hmodbus.rx_buf, hmodbus.rx_len - 2);
    uint16_t crc_recv = (hmodbus.rx_buf[hmodbus.rx_len - 2] << 8) | hmodbus.rx_buf[hmodbus.rx_len - 1];

    if (crc_calc != crc_recv)
        return; // CRC校验失败

    uint8_t addr = hmodbus.rx_buf[0];
    uint8_t func = hmodbus.rx_buf[1];

    // 地址校验
    if (addr != MODBUS_SLAVE_ADDR && addr != 0)
        return; // 广播地址0不响应

    // 处理功能码
    switch (func)
    {
    case 0x01: // Read Coils
        handle_read_coils();
        break;
    case 0x03: // Read Holding Registers
        handle_read_holding_regs();
        break;
    case 0x05: // Write Single Coil
        handle_write_coil();
        break;
    case 0x06: // Write Single Register
        handle_write_reg();
        break;
    case 0x10: // Write Multiple Registers
        handle_write_multiple_regs();
        break;
    default: // 非法功能码
        build_exception(func, 0x01);
        break;
    }

    // 发送响应
    if (addr != 0)
        RS485PowerTransmit(hmodbus.tx_buf, hmodbus.tx_len);

    // 重置状态
    hmodbus.rx_len = 0;
}

// Modbus初始化
void modbus_init(void)
{
    // 使能串口接收中断
    hmodbus.rx_len = 0;
    hmodbus.tx_len = 0;
    HAL_UART_Receive_IT(&MODBUS_USART, (uint8_t *)USART10_aRxBuffer, 1); // 开启接收中断
}
