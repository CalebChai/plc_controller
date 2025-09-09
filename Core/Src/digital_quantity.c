#include "digital_quantity.h"

/*
 * 10路数字量输出函数
 * digital_quantity：1~10的数字量
 * value：HIGH或LOW
 */
void DigitalQuantity_Output(uint8_t digital_quantity, uint8_t value)
{
    switch (digital_quantity)
    {
    case 1:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
        break;
    case 2:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
        break;
    case 3:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        break;
    case 4:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
        break;
    case 5:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        break;
    case 6:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
        break;
    case 7:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
        break;
    case 8:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
        break;
    case 9:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
        break;
    case 10:
        if (value == HIGH)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
        else if (value == LOW)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
        break;

    default:
        break;
    }
}

/*
 * 20路数字量读取函数
 * digital_quantity：1~20的数字量变量的地址
 */
uint8_t DigitalQuantity_Input(uint8_t digital_quantity)
{

    static uint8_t input_value;
    switch (digital_quantity)
    {
    case 1:
        input_value = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
        break;
    case 2:
        input_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
        break;
    case 3:
        input_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
        break;
    case 4:
        input_value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
        break;
    case 5:
        input_value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
        break;
    case 6:
        input_value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
        break;

    case 7:
        input_value = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
        break;
    case 8:
        input_value = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
        break;
    case 9:
        input_value = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);
        break;
    case 10:
        input_value = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);
        break;
    case 11:
        input_value = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);
        break;

    case 12:
        input_value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
        break;
    case 13:
        input_value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
        break;
    case 14:
        input_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
        break;
    case 15:
        input_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
        break;
    case 16:
        input_value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
        break;

    case 17:
        input_value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
        break;
    case 18:
        input_value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
        break;
    case 19:
        input_value = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
        break;
    case 20:
        input_value = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
        break;
    default:
        break;
    }
    return input_value;
}