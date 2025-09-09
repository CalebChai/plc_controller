#ifndef EEPROM_H
#define EEPROM_H
#include "i2c.h"
// ���� EEPROM ����������ʵ�������޸ģ�
#define EEPROM_I2C_ADDR 0x50 // 7λI2C������ַ
#define PAGE_SIZE 64         // 24C256��ҳ��СΪ64�ֽ�
uint8_t eepromWrite(uint16_t memAddr, uint8_t data);
uint8_t eepromRead(uint16_t memAddr);
uint8_t eepromPageWrite(uint16_t memAddr, uint8_t *data, uint8_t len);
uint16_t eepromSequentialRead(uint16_t startAddr, uint8_t *data, uint16_t len);
#endif