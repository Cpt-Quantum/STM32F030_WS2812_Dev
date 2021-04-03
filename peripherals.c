#include "inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

#include "peripherals.h"

void gpio_init(GPIO_TypeDef *gpio_port, const GPIO_PIN_E io_pin, GPIO_MODER_E gpio_mode,
				GPIO_ALT_MODE_E gpio_af)
{
	/* Enable the clock to the specified port */
	if (gpio_port == GPIOA)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	}
	else if (gpio_port == GPIOB)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	}
	else if (gpio_port == GPIOC)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	}
	else if (gpio_port == GPIOD)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIODEN;
	}
	else if (gpio_port == GPIOF)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
	}

	/* Set the GPIO mode for the specified pin */
	gpio_port->MODER &= ~(GPIO_MODER_MAX << (io_pin * 2));
	gpio_port->MODER |= (gpio_mode << (io_pin * 2));
	/* If the specified GPIO mode is for the alternate function, set the alternate function */
	if (gpio_mode == GPIO_ALT_MODE)
	{
		gpio_port->AFR[io_pin/8] &= ~(GPIO_AF_MAX << ((io_pin & 0x07) * 4));
		gpio_port->AFR[io_pin/8] |= (gpio_af << ((io_pin & 0x07) * 4));
	}
}

void gpio_output(GPIO_TypeDef *gpio_port, const GPIO_PIN_E gpio_pin, uint8_t value)
{
	/* Set the bit in the bit set/reset register */
	/* The reset bits are the upper 16 bits, and the set bits the lower 16 bits */
	/* This detail is handled by the shift left on input value logic */
	/* TODO: & value with 0x01 to ensure the input value is 1 or 0? */
	/* TODO: Alternatively make a GPIO pin value enum with only 1 or 0 as options */
	gpio_port->BSRR = ((1 << gpio_pin) << (16 * (!value))); 
}

void init_timer(TIM_TypeDef *TIMx)
{
	/* Make sure the timer is disabled before starting initialisation */
	TIMx->CR1 &= ~(TIM_CR1_CEN);

	/* Enable the clock to the timer and reset it */
	if (TIMx == TIM1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
		RCC->APB2RSTR |=   RCC_APB2RSTR_TIM1RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM1RST);
	}
	else if (TIMx == TIM16)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
		RCC->APB2RSTR |=   RCC_APB2RSTR_TIM16RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM16RST);
	}
	else if (TIMx == TIM17)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
		RCC->APB2RSTR |=   RCC_APB2RSTR_TIM17RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM17RST);
	}
	else if (TIMx == TIM3)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		RCC->APB1RSTR |=   RCC_APB1RSTR_TIM3RST;
		RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST);
	}
	else if (TIMx == TIM14)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
		RCC->APB1RSTR |=   RCC_APB1RSTR_TIM14RST;
		RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM14RST);
	}
}

void start_timer(TIM_TypeDef *TIMx, uint16_t prescale, uint16_t count)
{
	/* Set the prescaler */
	TIMx->PSC = prescale;
	/* Set the value to count to */
	TIMx->ARR = count;
	/* Set the update event to generate an update of its registers and reset it */
	TIMx->EGR |= TIM_EGR_UG;
	/* Setup timer to trigger a hardware interrupt upon reaching the count */
	TIMx->DIER |= TIM_DIER_UIE;
	/* Now enable the timer */
	TIMx->CR1 |= TIM_CR1_CEN;
}

void setup_timer_capture_compare(TIM_TypeDef *TIMx, const TIMER_CHANNEL_E channel,
		uint16_t ARR, uint16_t CCR, uint16_t prescale, bool flip_polarity, bool preload)
{
	/* Set prescaler */
	TIMx->PSC = prescale;
	/* Set the overall count */
	TIMx->ARR = ARR;

	switch(channel)
	{
		case TIM_CHAN_1:
			/* Set the first count in the capture compare register */
			TIMx->CCR1 = CCR;
			/* Set the output to go high when the CNT register reaches the value in */
			/* CCR1. */
			TIMx->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk);
			TIMx->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
			if (flip_polarity == true)
			{
				TIMx->CCMR1 |= (TIM_CCMR1_OC1M_0);
			}
			/* Set the preload for the capture compare value register */
			if (preload == true)
			{
				TIMx->CCMR1 |= TIM_CCMR1_OC1PE;
			}
			else
			{
				TIMx->CCMR1 &= TIM_CCMR1_OC1PE;
			}
			/* Disable the capture/compare 1 output for now*/
			/* NB: Can also set the polarity in this register */
			TIMx->CCER &= ~TIM_CCER_CC1E;
			break;
		case TIM_CHAN_2:
			/* Set the first count in the capture compare register */
			TIMx->CCR2 = CCR;
			/* Set the output to go high when the CNT register reaches the value in */
			/* CCR1. */
			TIMx->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk);
			TIMx->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
			if (flip_polarity == true)
			{
				TIMx->CCMR1 |= (TIM_CCMR1_OC2M_0);
			}
			/* Set the preload for the capture compare value register */
			if (preload == true)
			{
				TIMx->CCMR1 |= TIM_CCMR1_OC2PE;
			}
			else
			{
				TIMx->CCMR1 &= TIM_CCMR1_OC2PE;
			}
			/* Disable the capture/compare 1 output for now*/
			/* NB: Can also set the polarity in this register */
			TIMx->CCER &= ~TIM_CCER_CC2E;
			break;
		case TIM_CHAN_3:
			/* Set the first count in the capture compare register */
			TIMx->CCR3 = CCR;
			/* Set the output to go high when the CNT register reaches the value in */
			/* CCR1. */
			TIMx->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk);
			TIMx->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);
			if (flip_polarity == true)
			{
				TIMx->CCMR2 |= (TIM_CCMR2_OC3M_0);
			}
			/* Set the preload for the capture compare value register */
			if (preload == true)
			{
				TIMx->CCMR2 |= TIM_CCMR2_OC3PE;
			}
			else
			{
				TIMx->CCMR2 &= TIM_CCMR2_OC3PE;
			}
			/* Disable the capture/compare 1 output for now*/
			/* NB: Can also set the polarity in this register */
			TIMx->CCER &= ~TIM_CCER_CC3E;
			break;
		case TIM_CHAN_4:
			/* Set the first count in the capture compare register */
			TIMx->CCR4 = CCR;
			/* Set the output to go high when the CNT register reaches the value in */
			/* CCR1. */
			TIMx->CCMR2 &= ~(TIM_CCMR2_OC4M_Msk);
			TIMx->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);
			if (flip_polarity == true)
			{
				TIMx->CCMR2 |= (TIM_CCMR2_OC4M_0);
			}
			/* Set the preload for the capture compare value register */
			if (preload == true)
			{
				TIMx->CCMR2 |= TIM_CCMR2_OC4PE;
			}
			else
			{
				TIMx->CCMR2 &= TIM_CCMR2_OC4PE;
			}
			/* Disable the capture/compare 1 output for now*/
			/* NB: Can also set the polarity in this register */
			TIMx->CCER &= ~TIM_CCER_CC4E;
			break;
	}

	/* Set the update event to generate an update of its registers and reset it */
	//TIMx->EGR |= TIM_EGR_UG;
	/* Set the ARPE bit to allow changes to registers to take immediate effect */
	//TIMx->CR1 |= TIM_CR1_ARPE;
	/* Setup timer to trigger a hardware interrupt upon reaching the count */
	//TIMx->DIER |= TIM_DIER_UIE;
	/* Disable the timer for now*/
	TIMx->CR1 &= ~TIM_CR1_CEN;
}

void clock_setup(bool external_clk, bool use_pll, PLL_MULT_E pll_mult)
{
	/* Enable the selected clock and wait for it to be ready */
	if (external_clk == true)
	{
		RCC->CR |= RCC_CR_HSEON;
		while(!(RCC->CR & RCC_CR_HSERDY)){};
	}
	else
	{
		RCC->CR |= RCC_CR_HSION;
		while(!(RCC->CR & RCC_CR_HSIRDY)){};
	}

	/* Now enable the PLL if needed */
	if (use_pll == true)
	{
		if (external_clk == true)
		{
			RCC->CFGR &= ~(RCC_CFGR_PLLMUL |
						   RCC_CFGR_PLLSRC);
			RCC->CFGR |= (RCC_CFGR_PLLSRC_HSE_PREDIV |
						  pll_mult);
		}
		else
		{
			RCC->CFGR &= ~(RCC_CFGR_PLLMUL |
						   RCC_CFGR_PLLSRC);
			RCC->CFGR |= (RCC_CFGR_PLLSRC_HSI_DIV2 |
						  pll_mult);
		}

		/* Turn on the PLL and wait for the hardware to set the ready flag */
		RCC->CR |= RCC_CR_PLLON;
		while(!(RCC->CR & RCC_CR_PLLRDY)){};
	}

	/* Now set the clock source of the system clock */
	if (use_pll == true)
	{
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){};
	}
	else if (external_clk == true)
	{
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_HSE;
		while(!(RCC->CFGR & RCC_CFGR_SWS_HSE)){};
	}
	else if (external_clk == false)
	{
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_HSI;
		while(!(RCC->CFGR & RCC_CFGR_SWS_HSI)){};
	}
}

/* SysTick definitions */
volatile uint32_t systick = 0;

void delay_ms(uint32_t ms)
{
        uint32_t cycle_count = systick + ms;
        while(systick < cycle_count)
        {
                __asm__("WFI");
        }
}

