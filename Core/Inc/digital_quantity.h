#ifndef DIGITAL_QUANTITY_H
#define DIGITAL_QUANTITY_H
#include "main.h"
#include "stdint.h"
#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET
void DigitalQuantity_Output(uint8_t digital_quantity, uint8_t value);
uint8_t DigitalQuantity_Input(uint8_t digital_quantity);
#endif