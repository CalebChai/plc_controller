#ifndef EEPROM_H
#define EEPROM_H
#include "i2c.h"
// 定义 EEPROM 参数（根据实际器件修改）
#define EEPROM_I2C_ADDR 0x50 // 7位I2C器件地址
#define PAGE_SIZE 64         // 24C256的页大小为64字节
uint8_t eepromWrite(uint16_t memAddr, uint8_t data);
uint8_t eepromRead(uint16_t memAddr);
uint8_t eepromPageWrite(uint16_t memAddr, uint8_t *data, uint8_t len);
uint16_t eepromSequentialRead(uint16_t startAddr, uint8_t *data, uint16_t len);
#endif