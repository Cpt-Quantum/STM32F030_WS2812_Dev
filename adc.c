#include <stdint.h>

#include "inc/stm32f030x6.h"

#include "adc.h"

void adc_init(uint32_t channel_select, uint16_t *data, uint32_t data_size)
{
	/* Enable the clock to the ADC peripheral */
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	/* Enable the clock to the DMA peripheral */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	/* Set the clock source for the ADC to be PCLK/4 */
	ADC1->CFGR2 &= ~(ADC_CFGR2_CKMODE_1);
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE_1;
	/* Enable DMA transfer for ADC and enable circular mode */
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG | ADC_CFGR1_CONT;
	/* Select the active channels */
	ADC1->CHSELR = channel_select;

	/* DMA setup */
	/* Set the peripheral address for data to be transferred from */
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
	/* Set the address of the memory object to transfer ADC data into */
	DMA1_Channel1->CMAR = (uint32_t)(data);
	/* Set the size of the buffer */
	DMA1_Channel1->CNDTR = data_size;
	/* Set DMA config */
	DMA1_Channel1->CCR |= (DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 |
							DMA_CCR_TEIE | DMA_CCR_CIRC);
	/* Enable the DMA interrupts for channel 1 in the NVIC */

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

