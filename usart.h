#ifndef USART_H
#define USART_H
#include "inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

void usart_init(USART_TypeDef *USARTx, uint16_t prescaler,
				bool txe_interrupt_en, bool rxne_interrupt_en, 
				bool tc_interrupt_en, uint8_t interrupt_prio);

/* Simple function to combine all of the interrupts for USART into one number */
static inline uint32_t usart_interrupt_combine(bool txe_interrupt_en,
									  			bool rxne_interrupt_en, 
									  			bool tc_interrupt_en)
{
	return ((txe_interrupt_en ? USART_CR1_TXEIE:0)   |
			(rxne_interrupt_en ? USART_CR1_RXNEIE:0) |
			(tc_interrupt_en ? USART_CR1_TCIE:0));
}

static inline void usart_start_tx(USART_TypeDef *USARTx, 
				bool txe_interrupt_en, bool rxne_interrupt_en, 
				bool tc_interrupt_en)
{
	USARTx->CR1 |= (USART_CR1_TE | 
					usart_interrupt_combine(txe_interrupt_en,
											rxne_interrupt_en,
											tc_interrupt_en));
}
static inline void usart_stop_tx(USART_TypeDef *USARTx, 
				bool txe_interrupt_en, bool rxne_interrupt_en, 
				bool tc_interrupt_en)
{
	USARTx->CR1 &= ~(USART_CR1_TE | 
					usart_interrupt_combine(txe_interrupt_en,
											rxne_interrupt_en,
											tc_interrupt_en));
}

static inline void usart_start_rx(USART_TypeDef *USARTx, 
				bool txe_interrupt_en, bool rxne_interrupt_en, 
				bool tc_interrupt_en)
{
	USARTx->CR1 |= (USART_CR1_RE | 
					usart_interrupt_combine(txe_interrupt_en,
											rxne_interrupt_en,
											tc_interrupt_en));
}
static inline void usart_stop_rx(USART_TypeDef *USARTx, 
				bool txe_interrupt_en, bool rxne_interrupt_en, 
				bool tc_interrupt_en)
{
	USARTx->CR1 &= ~(USART_CR1_RE | 
					usart_interrupt_combine(txe_interrupt_en,
											rxne_interrupt_en,
											tc_interrupt_en));
}

#endif //USART_H
