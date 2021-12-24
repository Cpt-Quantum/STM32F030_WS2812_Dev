#include "../inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

#include "gpio.h"

void gpio_init(GPIO_TypeDef *GPIOx, const GPIO_PIN_E io_pin, GPIO_MODER_E gpio_mode,
			   GPIO_ALT_MODE_E gpio_af, GPIO_SPEED_E gpio_speed)
{
	/* Enable the clock to the specified port */
	if (GPIOx == GPIOA)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	}
	else if (GPIOx == GPIOB)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	}
	else if (GPIOx == GPIOC)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	}
	else if (GPIOx == GPIOD)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIODEN;
	}
	else if (GPIOx == GPIOF)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
	}

	/* Set the GPIO mode for the specified pin */
	GPIOx->MODER &= ~(GPIO_MODER_MAX << (io_pin * 2));
	GPIOx->MODER |= (gpio_mode << (io_pin * 2));
	/* If the specified GPIO mode is for the alternate function, set the alternate function */
	if (gpio_mode == GPIO_ALT_MODE)
	{
		GPIOx->AFR[io_pin / 8] &= ~(GPIO_AF_MAX << ((io_pin & 0x07) * 4));
		GPIOx->AFR[io_pin / 8] |= (gpio_af << ((io_pin & 0x07) * 4));
	}

	/* Set the gpio speed */
	GPIOx->OSPEEDR &= ~(gpio_speed << ((io_pin & 0x0F) * 2));
	GPIOx->OSPEEDR |= (gpio_speed << ((io_pin & 0x0F) * 2));
}

void gpio_output(GPIO_TypeDef *GPIOx, const GPIO_PIN_E gpio_pin, uint8_t value)
{
	/* Set the bit in the bit set/reset register */
	/* The reset bits are the upper 16 bits, and the set bits the lower 16 bits */
	/* This detail is handled by the shift left on input value logic */
	/* TODO: & value with 0x01 to ensure the input value is 1 or 0? */
	/* TODO: Alternatively make a GPIO pin value enum with only 1 or 0 as options */
	GPIOx->BSRR = ((1 << gpio_pin) << (16 * (!value)));
}

uint8_t gpio_read(GPIO_TypeDef *GPIOx, const GPIO_PIN_E gpio_pin)
{
	return ((GPIOx->IDR >> gpio_pin) & 0x00000001);
}
