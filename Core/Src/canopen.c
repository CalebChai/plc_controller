#include "canopen.h"
static FDCAN_TxHeaderTypeDef can_cfg;
static uint8_t local_node_id = 0;
// 初始化CANopen通信
CANopen_Status canopen_init(uint8_t node_id, FDCAN_TxHeaderTypeDef fdcan_cfg)
{
    can_cfg = fdcan_cfg;
    local_node_id = node_id;
    return CANOPEN_OK;
}

// 发送NMT命令
CANopen_Status canopen_send_nmt(uint8_t node_id, uint8_t command)
{
    uint8_t nmt_data[8] = {command, node_id, 0, 0, 0, 0, 0, 0};
    return (FDCAN2_Send_Msg(nmt_data, 8, 0x000) == HAL_OK) ? CANOPEN_OK : CANOPEN_TX_ERROR;
}

// SDO下载请求（写对象字典）
CANopen_Status canopen_sdo_write(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t data, uint8_t len)
{
    static uint16_t len_cmd = 0;
    switch (len)
    {
    case 1:
        /* code */
        len_cmd = 0x0f; // 写1字节

        break;
    case 2:
        /* code */
        len_cmd = 0x0b; // 写2字节
        break;
    case 3:
        /* code */
        len_cmd = 0x07; // 写3字节
        break;
    case 4:
        /* code */
        len_cmd = 0x03; // 写4字节
        break;

    default:
        break;
    }
    uint8_t sdo_data[8] = {
        SDO_DOWNLOAD_REQ | len_cmd, // 命令字: 快速下载(4字节)
        index & 0xFF,               // 索引低字节
        index >> 8,                 // 索引高字节
        subindex,                   // 子索引
        (uint8_t)(data),            // 数据字节0
        (uint8_t)(data >> 8),       // 数据字节1
        (uint8_t)(data >> 16),      // 数据字节2
        (uint8_t)(data >> 24)       // 数据字节3
    };

    // 设置目标节点SDO COB-ID
    uint32_t original_tx_id = can_cfg.Identifier;
    can_cfg.Identifier = 0x600 + node_id;

    HAL_StatusTypeDef status = FDCAN2_Send_Msg(sdo_data, 8, can_cfg.Identifier);

    // 恢复原始ID
    can_cfg.Identifier = original_tx_id;

    return (status == HAL_OK) ? CANOPEN_OK : CANOPEN_TX_ERROR;
}

// SDO读取请求（读对象字典）
CANopen_Status canopen_sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex)
{
    static uint16_t len_cmd = 0;
    uint8_t sdo_data[8] = {
        0x40,         // 命令字: 读取
        index & 0xFF, // 索引低字节
        index >> 8,   // 索引高字节
        subindex,     // 子索引
        0x00,         // 数据字节0
        0x00,         // 数据字节1
        0x00,         // 数据字节2
        0x00          // 数据字节3
    };

    // 设置目标节点SDO COB-ID
    uint32_t original_tx_id = can_cfg.Identifier;
    can_cfg.Identifier = 0x600 + node_id;

    HAL_StatusTypeDef status = FDCAN2_Send_Msg(sdo_data, 8, can_cfg.Identifier);

    // 恢复原始ID
    can_cfg.Identifier = original_tx_id;

    return (status == HAL_OK) ? CANOPEN_OK : CANOPEN_TX_ERROR;
}

// CAN接收数据处理
void canopen_process_rx(uint32_t id, uint8_t *data, uint8_t len)
{
    // 1. 处理NMT响应
    if (id == 0x700 + local_node_id)
    {
        uint8_t state = data[0];
        // 状态机处理...
    }

    // 2. 处理SDO响应
    else if ((id & 0x580) == 0x580)
    {
        uint8_t cmd = data[0];
        uint16_t index = data[1] | (data[2] << 8);
        uint8_t subindex = data[3];

        // 处理SDO响应...
    }

    // 3. 处理PDO
    else if ((id >= 0x180) && (id <= 0x57F))
    {
        // PDO数据处理...
    }
}