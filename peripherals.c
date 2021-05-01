#include "inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

#include "peripherals.h"

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

