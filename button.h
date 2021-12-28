#ifndef BUTTON_H
#define BUTTON_H

#include "inc/stm32f030x6.h"

#include <stdbool.h>
#include <stdint.h>
#include "headers/gpio.h"

typedef struct
{
	GPIO_TypeDef *GPIOx;
	GPIO_PIN_E pin_num;
	uint16_t accumulator;
} button_t;

bool button_state_update(button_t *button);

#endif /* BUTTON_H */