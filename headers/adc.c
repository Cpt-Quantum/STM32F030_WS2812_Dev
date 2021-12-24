#include <stdint.h>
#include <stddef.h>

#include "../inc/stm32f030x6.h"

#include "adc.h"
#include "timer.h"

ADC_t *ADC_local = NULL;

void adc_init(ADC_t *adc_settings)
{
	/* Enable the clock to the ADC peripheral */
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	/* Assign a pointer to the passed ADC_t struct to the local pointer */
	ADC_local = adc_settings;

	/* Calibrate the ADC */
	/* This code is taken almost verbatim from the STM32F030 reference manual */
	if ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		ADC1->CR |= ADC_CR_ADDIS;
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) != 0)
	{
	}

	/* Enable the clock to the DMA peripheral */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	/* Set the clock source for the ADC to be PCLK/4 */
	ADC1->CFGR2 &= ~(ADC_CFGR2_CKMODE_1 | ADC_CFGR2_CKMODE_0);
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE_1;
	/* Enable DMA transfer for ADC and enable circular mode */
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;
	/* Select the active channels */
	ADC1->CHSELR = adc_settings->channel_select;
	/* Enable external trigger on rising edge and select TIM1_TRGO as */
	/* external trigger for ADC conversion. */
	ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0;

	/* Enable timer 1 */
	init_timer(TIM1);
	TIM1->PSC = 0;
	TIM1->ARR = (adc_settings->clock_div - 1);
	/* Set the master mode selection of TIM1 to generate trigger output (TRGO) */
	/* on update events that occur when the counter reaches the value in ARR   */
	TIM1->CR2 = TIM_CR2_MMS_1;
	TIM1->CR1 |= TIM_CR1_CEN;

	/* DMA setup */
	/* Set the peripheral address for data to be transferred from */
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
	/* Set the address of the memory object to transfer ADC data into */
	DMA1_Channel1->CMAR = (uint32_t)(adc_settings->data);
	/* Set the size of the buffer */
	DMA1_Channel1->CNDTR = adc_settings->data_length;
	/* Set DMA config */
	DMA1_Channel1->CCR |= (DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 |
						   DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE |
						   DMA_CCR_CIRC);
	/* Enable the DMA interrupts for channel 1 in the NVIC */
	NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	/* Enable DMA */
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	/* Enable the ADC */
	/* Making sure that the ADC is ready before setting the enable */
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)
	{
		ADC1->ISR |= ADC_ISR_ADRDY;
	}
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
	{
		/* For robust implementation, add here time-out management */
	}
	/* Now start the ADC conversion */
	ADC1->CR |= ADC_CR_ADSTART;
}

/* This DMA interrupt handler is used by the ADC to copy data to a buffer */
void DMA1_Channel1_IRQHandler(void)
{
	/* Half way through buffer interrupt */
	if (DMA1->ISR & DMA_ISR_HTIF1)
	{
		/* Clear the interrupt flag */
		DMA1->IFCR = DMA_IFCR_CHTIF1;
		/* Jump to the callback function */
		ADC_local->callback(ADC_BUFFER_HALF_FULL);
	}
	/* End of buffer interrupt */
	else if (DMA1->ISR & DMA_ISR_TCIF1)
	{
		/* Clear the interrupt flag */
		DMA1->IFCR = DMA_IFCR_CTCIF1;
		/* Jump to the callback function */
		ADC_local->callback(ADC_BUFFER_FULL);
	}
	if (DMA1->ISR & DMA_ISR_TEIF1)
	{
		/* Transfer error interrupt flag */
		/* TODO: Actually handle this properly, for now just clear it */
		DMA1->IFCR = DMA_IFCR_CTEIF1;
	}
}