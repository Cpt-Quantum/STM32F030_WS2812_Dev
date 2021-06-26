#ifndef USART_H
#define USART_H
#include "inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

#include "circular_buffer.h"

/* Type for handling the clock source of the USART */
typedef enum
{
	USART_CLK_PCLK = RCC_CFGR3_USART1SW_PCLK,
	USART_CLK_SYSCLK = RCC_CFGR3_USART1SW_SYSCLK,
	USART_CLK_LSECLK = RCC_CFGR3_USART1SW_LSE,
	USART_CLK_HSICLK = RCC_CFGR3_USART1SW_HSI
} USART_CLK_SRC_E;

/* Struct to hold all initialisation information/settings for a particular USART */
typedef struct {
	USART_TypeDef *USARTx;
	USART_CLK_SRC_E clk_src;
	uint16_t prescaler;
	bool txe_interrupt_en;
	bool rxne_interrupt_en;
	bool tc_interrupt_en;
	uint8_t interrupt_prio;
	ringbuffer_t *tx_buffer;
	ringbuffer_t *rx_buffer;
} USART_t;

/* Function to initialise the USART based on the settings in the provided struct */
void usart_init(USART_t USART_settings);
/* Function to write string into USART TX ringbuffer */
void usart_write_tx_buffer(USART_t USART_settings, char* string, uint32_t length);
/* Function to read current contents of the RX buffer into the provided string */
void usart_read_rx_buffer(USART_t USART_settings, char *string, uint32_t length);

/* Simple function to combine all of the interrupts for USART into one number */
static inline uint32_t usart_interrupt_combine(bool txe_interrupt_en,
									  			bool rxne_interrupt_en, 
									  			bool tc_interrupt_en)
{
	return ((txe_interrupt_en ? USART_CR1_TXEIE:0)   |
			(rxne_interrupt_en ? USART_CR1_RXNEIE:0) |
			(tc_interrupt_en ? USART_CR1_TCIE:0));
}

static inline void usart_start_tx(USART_t USART_settings)
{
	USART_settings.USARTx->CR1 |= (USART_CR1_TE |
									usart_interrupt_combine(
											USART_settings.txe_interrupt_en,
											USART_settings.rxne_interrupt_en,
											USART_settings.tc_interrupt_en));
}
static inline void usart_stop_tx(USART_t USART_settings)
{
	USART_settings.USARTx->CR1 &= ~(USART_CR1_TE |
									usart_interrupt_combine(
											USART_settings.txe_interrupt_en,
											0,
											USART_settings.tc_interrupt_en));

}

static inline void usart_start_rx(USART_t USART_settings)
{
	USART_settings.USARTx->CR1 |= (USART_CR1_RE |
									usart_interrupt_combine(
											USART_settings.txe_interrupt_en,
											USART_settings.rxne_interrupt_en,
											USART_settings.tc_interrupt_en));

}
static inline void usart_stop_rx(USART_t USART_settings)
{
	USART_settings.USARTx->CR1 &= ~(USART_CR1_RE |
									usart_interrupt_combine(
											0,
											USART_settings.rxne_interrupt_en,
											0));
}

#endif //USART_H
