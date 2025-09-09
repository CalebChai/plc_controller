#ifndef CANOPEN_H
#define CANOPEN_H
#include "fdcan.h"

// CANopen������
#define NMT_START_CMD 0x01
#define SDO_UPLOAD_REQ 0x40   // ���ֵ�
#define SDO_DOWNLOAD_REQ 0x20 // д�ֵ�

// CANopen�ڵ�״̬
typedef enum
{
    NMT_PRE_OPERATIONAL = 0x7F,
    NMT_OPERATIONAL = 0x05,
    NMT_STOPPED = 0x04
} NMT_State;

// CANopen������
typedef enum
{
    CANOPEN_OK = 0,
    CANOPEN_TX_ERROR,
    CANOPEN_TIMEOUT
} CANopen_Status;

// ��������
CANopen_Status canopen_init(uint8_t node_id, FDCAN_TxHeaderTypeDef fdcan_cfg);
CANopen_Status canopen_send_nmt(uint8_t node_id, uint8_t command);
CANopen_Status canopen_sdo_write(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t data, uint8_t len);
CANopen_Status canopen_sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex);
void canopen_process_rx(uint32_t id, uint8_t *data, uint8_t len);
#endif