#ifndef GPIO_H
#define GPIO_H
#include "../inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	GPIO_INPUT = 0,
	GPIO_OUTPUT = 1,
	GPIO_ALT_MODE = 2,
	GPIO_ANALOGUE = 3,
	GPIO_MODER_MAX = 3
} GPIO_MODER_E;

typedef enum
{
	PIN_0 = 0,
	PIN_1 = 1,
	PIN_2 = 2,
	PIN_3 = 3,
	PIN_4 = 4,
	PIN_5 = 5,
	PIN_6 = 6,
	PIN_7 = 7,
	PIN_8 = 8,
	PIN_9 = 9,
	PIN_10 = 10,
	PIN_11 = 11,
	PIN_12 = 12,
	PIN_13 = 13,
	PIN_14 = 14,
	PIN_15 = 15
} GPIO_PIN_E;

typedef enum
{
	GPIO_AF0 = 0,
	GPIO_AF1 = 1,
	GPIO_AF2 = 2,
	GPIO_AF3 = 3,
	GPIO_AF4 = 4,
	GPIO_AF5 = 5,
	GPIO_AF6 = 6,
	GPIO_AF7 = 7,
	GPIO_AF_MAX = 0xF
} GPIO_ALT_MODE_E;

typedef enum
{
	GPIO_LOW_SPEED = 0,
	GPIO_MED_SPEED = 1,
	GPIO_HIGH_SPEED = 3
} GPIO_SPEED_E;

void gpio_init(GPIO_TypeDef *GPIOx, const GPIO_PIN_E io_pin, GPIO_MODER_E gpio_mode,
			   GPIO_ALT_MODE_E gpio_af, GPIO_SPEED_E gpio_speed);

void gpio_output(GPIO_TypeDef *GPIOx, const GPIO_PIN_E io_pin, uint8_t value);
uint8_t gpio_read(GPIO_TypeDef *GPIOx, const GPIO_PIN_E io_pin);

#endif // GPIO_H
