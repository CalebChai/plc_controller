#include "canopen.h"
static FDCAN_TxHeaderTypeDef can_cfg;
static uint8_t local_node_id = 0;
// ��ʼ��CANopenͨ��
CANopen_Status canopen_init(uint8_t node_id, FDCAN_TxHeaderTypeDef fdcan_cfg)
{
    can_cfg = fdcan_cfg;
    local_node_id = node_id;
    return CANOPEN_OK;
}

// ����NMT����
CANopen_Status canopen_send_nmt(uint8_t node_id, uint8_t command)
{
    uint8_t nmt_data[8] = {command, node_id, 0, 0, 0, 0, 0, 0};
    return (FDCAN2_Send_Msg(nmt_data, 8, 0x000) == HAL_OK) ? CANOPEN_OK : CANOPEN_TX_ERROR;
}

// SDO��������д�����ֵ䣩
CANopen_Status canopen_sdo_write(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t data, uint8_t len)
{
    static uint16_t len_cmd = 0;
    switch (len)
    {
    case 1:
        /* code */
        len_cmd = 0x0f; // д1�ֽ�

        break;
    case 2:
        /* code */
        len_cmd = 0x0b; // д2�ֽ�
        break;
    case 3:
        /* code */
        len_cmd = 0x07; // д3�ֽ�
        break;
    case 4:
        /* code */
        len_cmd = 0x03; // д4�ֽ�
        break;

    default:
        break;
    }
    uint8_t sdo_data[8] = {
        SDO_DOWNLOAD_REQ | len_cmd, // ������: ��������(4�ֽ�)
        index & 0xFF,               // �������ֽ�
        index >> 8,                 // �������ֽ�
        subindex,                   // ������
        (uint8_t)(data),            // �����ֽ�0
        (uint8_t)(data >> 8),       // �����ֽ�1
        (uint8_t)(data >> 16),      // �����ֽ�2
        (uint8_t)(data >> 24)       // �����ֽ�3
    };

    // ����Ŀ��ڵ�SDO COB-ID
    uint32_t original_tx_id = can_cfg.Identifier;
    can_cfg.Identifier = 0x600 + node_id;

    HAL_StatusTypeDef status = FDCAN2_Send_Msg(sdo_data, 8, can_cfg.Identifier);

    // �ָ�ԭʼID
    can_cfg.Identifier = original_tx_id;
    printf("status:%d",status);
    return (status == HAL_OK) ? CANOPEN_OK : CANOPEN_TX_ERROR;
}

// SDO��ȡ���󣨶������ֵ䣩
CANopen_Status canopen_sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex)
{
    static uint16_t len_cmd = 0;
    uint8_t sdo_data[8] = {
        0x40,         // ������: ��ȡ
        index & 0xFF, // �������ֽ�
        index >> 8,   // �������ֽ�
        subindex,     // ������
        0x00,         // �����ֽ�0
        0x00,         // �����ֽ�1
        0x00,         // �����ֽ�2
        0x00          // �����ֽ�3
    };

    // ����Ŀ��ڵ�SDO COB-ID
    uint32_t original_tx_id = can_cfg.Identifier;
    can_cfg.Identifier = 0x600 + node_id;

    HAL_StatusTypeDef status = FDCAN2_Send_Msg(sdo_data, 8, can_cfg.Identifier);

    // �ָ�ԭʼID
    can_cfg.Identifier = original_tx_id;

    return (status == HAL_OK) ? CANOPEN_OK : CANOPEN_TX_ERROR;
}

// CAN�������ݴ���
void canopen_process_rx(uint32_t id, uint8_t *data, uint8_t len)
{
    // 1. ����NMT��Ӧ
    if (id == 0x700 + local_node_id)
    {
        uint8_t state = data[0];
        // ״̬������...
    }

    // 2. ����SDO��Ӧ
    else if ((id & 0x580) == 0x580)
    {
        uint8_t cmd = data[0];
        uint16_t index = data[1] | (data[2] << 8);
        uint8_t subindex = data[3];

        // ����SDO��Ӧ...
    }

    // 3. ����PDO
    else if ((id >= 0x180) && (id <= 0x57F))
    {
        // PDO���ݴ���...
    }
}