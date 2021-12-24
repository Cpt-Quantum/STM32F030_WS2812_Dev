#include "../inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

#include "timer.h"

void init_timer(TIM_TypeDef *TIMx)
{
	/* Make sure the timer is disabled before starting initialisation */
	TIMx->CR1 &= ~(TIM_CR1_CEN);

	/* Enable the clock to the timer and reset it */
	if (TIMx == TIM1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM1RST);
	}
	else if (TIMx == TIM16)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_TIM16RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM16RST);
	}
	else if (TIMx == TIM17)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_TIM17RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM17RST);
	}
	else if (TIMx == TIM3)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
		RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST);
	}
	else if (TIMx == TIM14)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
		RCC->APB1RSTR |= RCC_APB1RSTR_TIM14RST;
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

	switch (channel)
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
