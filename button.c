#include "inc/stm32f030x6.h"

#include <stdbool.h>
#include <stdint.h>
#include "headers/gpio.h"
#include "button.h"

bool button_state_update(button_t *button)
{
	button->accumulator = (button->accumulator << 1) | gpio_read(button->GPIOx, button->pin_num) | 0xE000;

	if (button->accumulator == 0xF000)
	{
		return true;
	}
	return false;
}