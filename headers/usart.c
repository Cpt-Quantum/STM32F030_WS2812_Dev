#include "../inc/stm32f030x6.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "usart.h"
#include "circular_buffer.h"

USART_t *USART1_local = NULL;

void usart_init(USART_t *USART_settings)
{
	/* Enable the clock to the specified USART and reset it */
	if (USART_settings->USARTx == USART1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART1RST);
	}

	/* Set the clock source for the USART */
	RCC->CFGR3 &= ~(RCC_CFGR3_USART1SW_Msk);
	RCC->CFGR3 |= USART_settings->clk_src;

	/* Copy the pointer to the USART_settings->object to a local copy */
	USART1_local = USART_settings;

	/* Set the baud rate */
	USART_settings->USARTx->BRR = USART_settings->prescaler;

	/* If any interrupts are enabled, set the NVIC priority */
	if (
		USART_settings->txe_interrupt_en ||
		USART_settings->rxne_interrupt_en ||
		USART_settings->tc_interrupt_en)
	{
		NVIC_SetPriority(USART1_IRQn, USART_settings->interrupt_prio);
		NVIC_EnableIRQ(USART1_IRQn);
	}

	/* Now enable the peripheral */
	USART_settings->USARTx->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);

	/* Enable any specified interrupts */
	USART_settings->USARTx->CR1 |= usart_interrupt_combine(
		USART_settings->txe_interrupt_en,
		USART_settings->rxne_interrupt_en,
		USART_settings->tc_interrupt_en);
}

void usart_write_tx_buffer(USART_t USART_settings, char *string, uint32_t length)
{
	ringbuffer_write_string(USART_settings.tx_buffer, string, length);
}

void usart_read_rx_buffer(USART_t USART_settings, char *string, uint32_t length)
{
	ringbuffer_read_string(USART_settings.rx_buffer, string, length);
}

void USART1_IRQHandler(void)
{
	/* Check for TX register empty interrupt flag */
	if (USART1->ISR & USART_ISR_TXE)
	{
		/* Output next character in the ringbuffer */
		char temp = ringbuffer_read(USART1_local->tx_buffer);
		/* Check for an empty buffer, if so disable the transmit */
		if (USART1_local->tx_buffer->buffer_empty)
		{
			/* Disable the transmit buffer empty interrupt */
			/* This means we don't have to clear it */
			USART1->CR1 &= ~(USART_CR1_TXEIE);
		}
		else
		{
			/* Only write to the transfer register if the buffer isn't empty */
			USART1->TDR = temp;
		}
	}
	/* Check for RX register not empty interrupt flag */
	if (USART1->ISR & USART_ISR_RXNE)
	{
		char temp = USART1->RDR;
		ringbuffer_write(USART1_local->rx_buffer, temp);
	}
	/* Check for transmit complete interrupt flag */
	if (USART1->ISR & USART_ISR_TC)
	{
		///* Clear the interrupt flag */
		USART1->ICR = USART_ICR_TCCF;
	}
	/* Check for overrun error interrupt flag and clear it if it has tripped */
	if (USART1->ISR & USART_ISR_ORE)
	{
		USART1->ICR = USART_ICR_ORECF;
	}
}