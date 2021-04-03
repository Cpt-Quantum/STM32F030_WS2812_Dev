#ifndef PERIPHERALS_H
#define PERIPHERALS_H
#include "inc/stm32f030x6.h"

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
	PIN_0  =  0,
	PIN_1  =  1,
	PIN_2  =  2,
	PIN_3  =  3,
	PIN_4  =  4,
	PIN_5  =  5,
	PIN_6  =  6,
	PIN_7  =  7,
	PIN_8  =  8,
	PIN_9  =  9,
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

typedef enum {
	PLL_MULT_X2  = RCC_CFGR_PLLMUL2,
	PLL_MULT_X3  = RCC_CFGR_PLLMUL3,
	PLL_MULT_X4  = RCC_CFGR_PLLMUL4,
	PLL_MULT_X5  = RCC_CFGR_PLLMUL5,
	PLL_MULT_X6  = RCC_CFGR_PLLMUL6,
	PLL_MULT_X7  = RCC_CFGR_PLLMUL7,
	PLL_MULT_X8  = RCC_CFGR_PLLMUL8,
	PLL_MULT_X9  = RCC_CFGR_PLLMUL9,
	PLL_MULT_X10 = RCC_CFGR_PLLMUL10,
	PLL_MULT_X11 = RCC_CFGR_PLLMUL11,
	PLL_MULT_X12 = RCC_CFGR_PLLMUL12,
	PLL_MULT_X13 = RCC_CFGR_PLLMUL13,
	PLL_MULT_X14 = RCC_CFGR_PLLMUL14,
	PLL_MULT_X15 = RCC_CFGR_PLLMUL15,
	PLL_MULT_X16 = RCC_CFGR_PLLMUL16
} PLL_MULT_E;

typedef enum
{
	TIM_CHAN_1 = 0,
	TIM_CHAN_2 = 1,
	TIM_CHAN_3 = 2,
	TIM_CHAN_4 = 3,
} TIMER_CHANNEL_E;

extern volatile uint32_t systick;

void gpio_init(GPIO_TypeDef *gpio_port, const GPIO_PIN_E io_pin, GPIO_MODER_E gpio_mode,
				GPIO_ALT_MODE_E gpio_af);

void gpio_output(GPIO_TypeDef *gpio_port, const GPIO_PIN_E io_pin, uint8_t value);

void init_timer(TIM_TypeDef *TIMx);

void start_timer(TIM_TypeDef *TIMx, uint16_t prescale, uint16_t count);

void setup_timer_capture_compare(TIM_TypeDef *TIMx, const TIMER_CHANNEL_E channel,
		uint16_t ARR, uint16_t CCR, uint16_t prescale, bool flip_polarity, bool preload);

void clock_setup(bool external_clk, bool use_pll, PLL_MULT_E pll_mult);

void delay_ms(uint32_t ms);

#endif //PERIPHERALS_H
