#include "inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

#include "usart.h"

void usart_init(USART_TypeDef *USARTx, uint16_t prescaler,
				bool txe_interrupt_en, bool rxne_interrupt_en,
				bool tc_interrupt_en, uint8_t interrupt_prio)
{
	/* Enable the clock to the specified USART and reset it */
	if (USARTx == USART1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		RCC->APB2RSTR |=   RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART1RST);
	}
	/* Set the baud rate */
	USARTx->BRR = prescaler;

	/* If any interrupts are enabled, set the NVIC priority */
	if (txe_interrupt_en || rxne_interrupt_en || tc_interrupt_en)
	{
		NVIC_SetPriority(USART1_IRQn, interrupt_prio);
		NVIC_EnableIRQ(USART1_IRQn);
	}

	/* Now enable the peripheral */
	USARTx->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);

	/* Enable any specified interrupts */
	USARTx->CR1 |= usart_interrupt_combine(txe_interrupt_en, rxne_interrupt_en,
											tc_interrupt_en);

}
