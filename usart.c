#include "inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

#include "usart.h"
#include "circular_buffer.h"

void usart_init(USART_t USART_settings)
{
	/* Enable the clock to the specified USART and reset it */
	if (USART_settings.USARTx == USART1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		RCC->APB2RSTR |=   RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART1RST);
	}
	/* Set the baud rate */
	USART_settings.USARTx->BRR = USART_settings.prescaler;

	/* If any interrupts are enabled, set the NVIC priority */
	if (
		USART_settings.txe_interrupt_en ||
		USART_settings.rxne_interrupt_en ||
		USART_settings.tc_interrupt_en
		)
	{
		NVIC_SetPriority(USART1_IRQn, USART_settings.interrupt_prio);
		NVIC_EnableIRQ(USART1_IRQn);
	}

	/* Now enable the peripheral */
	USART_settings.USARTx->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);

	/* Enable any specified interrupts */
	USART_settings.USARTx->CR1 |= usart_interrupt_combine(
											USART_settings.txe_interrupt_en,
											USART_settings.rxne_interrupt_en,
											USART_settings.tc_interrupt_en);
}

void usart_write_tx_buffer(USART_t USART_settings, char *string, uint32_t length)
{
	ringbuffer_write_string(USART_settings.tx_buffer, string, length);
}

void usart_read_rx_buffer(USART_t USART_settings, char *string, uint32_t length)
{
	ringbuffer_read_string(USART_settings.rx_buffer, string, length);
}
