#include "inc/stm32f030x6.h"

#include "circular_buffer.h"
#include "peripherals.h"
#include "gpio.h"
#include "adc.h"
#include "timer.h"
#include "usart.h"
#include "ws2812.h"

#include <stdint.h>
#include <stdbool.h>

/* Set whether the board uses RGB or RGBW leds */
//#define RGBW

/* Define pin connections */
#define LED_PIN 0

#define NUM_LEDS 12
#define MAX_BRIGHTNESS 10	//NB: Changed from 180 to avoid blinding me

/* Define the LED data structure and point it at the LED data array */
#ifdef RGB
	#define LED_BYTES 3
	uint8_t led_data[NUM_LEDS * LED_BYTES];
	led_t leds = {
					.data = led_data,
					.bit_mask = 0B10000000,
					.arr_pos = 0,
					.num_leds = NUM_LEDS,
					.data_format = LED_GRB,
					.led_bytes = LED_BYTES,
				 };
#endif
#ifdef RGBW
	#define LED_BYTES 4
	uint8_t led_data[NUM_LEDS * LED_BYTES];
	led_t leds = {
					.data = led_data,
					.bit_mask = 0B10000000,
					.arr_pos = 0,
					.num_leds = NUM_LEDS,
					.data_format = LED_RGBW,
					.led_bytes = LED_BYTES,
				 };
#endif

#define SYSCLK_FREQ 48000000

/* Message to print over USART every second */
char usart_message[] = "Hello world!\n";

volatile char rx_char_buff = '\0';

/* Register the needed ringbuffers for USART1 RX and TX */
#define USART_BUFFER_LENGTH 32
volatile char usart_tx_buf[USART_BUFFER_LENGTH];
volatile char usart_rx_buf[USART_BUFFER_LENGTH];
ringbuffer_t usart_tx_buffer = {
	.buffer = usart_tx_buf,
	.head = 0,
	.tail = 0,
	.length = sizeof(usart_tx_buf),
	.buffer_empty = true,
	.buffer_full = false
};
ringbuffer_t usart_rx_buffer = {
	.buffer = usart_rx_buf,
	.head = 0,
	.tail = 0,
	.length = sizeof(usart_rx_buf),
	.buffer_empty = true,
	.buffer_full = false
};

/* Initialise the struct for USART1 settings */
USART_t USART1_settings = {
		.USARTx = USART1,
		.prescaler = SYSCLK_FREQ/9600,
		.txe_interrupt_en = true,
		.rxne_interrupt_en = true,
		.tc_interrupt_en = false,
		.interrupt_prio = 3,
		.tx_buffer = &usart_tx_buffer,
		.rx_buffer = &usart_rx_buffer
};

/* ADC data buffer */
#define ADC_BUFFER_SIZE 12
uint16_t adc_data[ADC_BUFFER_SIZE];

int main(void)
{
	/* Initialisation */

	/* Setup flash wait cycles */
	FLASH->ACR &= ~(0x00000017);
	FLASH->ACR |= (FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);

	/* Initialise system clock */
	clock_setup(false, true, PLL_MULT_X12);

#if defined RGB || defined RGBW
	/* Initialise the timer */
	init_timer(TIM3);
#endif

	/* Turn the LED on */
	gpio_init(GPIOA, LED_PIN, GPIO_OUTPUT, GPIO_AF0, GPIO_LOW_SPEED);
	gpio_output(GPIOA, LED_PIN, 1);

	/* Set up the USART pins for alternate functions */
	gpio_init(GPIOA, PIN_2, GPIO_ALT_MODE, GPIO_AF1, GPIO_HIGH_SPEED);
	gpio_init(GPIOA, PIN_3, GPIO_ALT_MODE, GPIO_AF1, GPIO_HIGH_SPEED);

	/* Set up PA6 for analogue input */
	gpio_init(GPIOA, PIN_6, GPIO_ANALOGUE, GPIO_AF0, GPIO_LOW_SPEED);
	/* Now enable the ADC */
	uint32_t channel_select = 1 << 5;
	adc_init(channel_select, adc_data, sizeof(adc_data)/sizeof(uint16_t));

	/* Enable PB1 with alternate functionality */
	gpio_init(GPIOB, PIN_1, GPIO_ALT_MODE, GPIO_AF1, GPIO_HIGH_SPEED);

	/* Enable USART1 */
	usart_init(USART1_settings);

	/* Setup the SysTick peripheral for 1ms ticks */
	SysTick_Config(SYSCLK_FREQ/1000);

	/* End of initialisation */

	/* Delay until LEDs are ready */
	delay_ms(100);

#if defined RGB || defined RGBW
	/* Enable DMA and initialise the LEDs */
	led_init();
#endif

#ifdef RGB
	led_rgb_write_all(&leds, 0, 0, 0);
	led_show(&leds, TIM3);
#endif
#ifdef RGBW
	led_rgbw_write_all(&leds, 0, 0, 0, 0);
	led_show(&leds, TIM3);
#endif

	/* Loop forever */
	while(1)
	{
		//char received_message[USART_BUFFER_LENGTH];
		//usart_read_rx_buffer(USART1_settings, received_message);
		char adc_usart_buffer[ADC_BUFFER_SIZE * 2];
		for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++)
		{
			//adc_usart_buffer[i]   = (char)(adc_data[i] & 0x00FF);
			//adc_usart_buffer[2*i] = (char)((adc_data[i] & 0xFF00) >> 8);
			adc_usart_buffer[i] = 'H';
			adc_usart_buffer[2*i] = 'W';
		}
		//usart_write_tx_buffer(USART1_settings, adc_usart_buffer, ADC_BUFFER_SIZE*2);
		usart_write_tx_buffer(USART1_settings, usart_message, (sizeof(usart_message) - 1));
		usart_start_tx(USART1_settings);
		gpio_output(GPIOA, LED_PIN, 1);
		delay_ms(1000);
		gpio_output(GPIOA, LED_PIN, 0);
		delay_ms(1000);
#ifdef RGB
		/* Set all LEDs to red and send out the data */
		led_rgb_write_all(&leds, MAX_BRIGHTNESS, 0, 0);
		led_show(&leds, TIM3);

		/* Wait 1s */
		delay_ms(1000);

		/* Set all LEDs to green and send out the data */
		led_rgb_write_all(&leds, 0, MAX_BRIGHTNESS, 0);
		led_show(&leds, TIM3);

		/* Wait 1s */
		delay_ms(1000);

		/* Set all LEDs to blue and send out the data */
		led_rgb_write_all(&leds, 0, 0, MAX_BRIGHTNESS);
		led_show(&leds, TIM3);

		/* Wait 1s */
		delay_ms(1000);
#endif
#ifdef RGBW
	/* Set all LEDs to red and send out the data */
		led_rgbw_write_all(&leds, MAX_BRIGHTNESS, 0, 0, 0);
		led_show(&leds, TIM3);

		/* Wait 1s */
		delay_ms(1000);

		/* Set all LEDs to green and send out the data */
		led_rgbw_write_all(&leds, 0, MAX_BRIGHTNESS, 0, 0);
		led_show(&leds, TIM3);

		/* Wait 1s */
		delay_ms(1000);

		/* Set all LEDs to blue and send out the data */
		led_rgbw_write_all(&leds, 0, 0, MAX_BRIGHTNESS, 0);
		led_show(&leds, TIM3);

		/* Wait 1s */
		delay_ms(1000);

		/* Set all LEDs to white and send out the data */
		led_rgbw_write_all(&leds, 0, 0, 0, MAX_BRIGHTNESS);
		led_show(&leds, TIM3);

		/* Wait 1s */
		delay_ms(1000);

#endif
	};
}

void TIM3_IRQHandler(void)
{
	/* Check the cause of the interrupt */
	if (TIM3->SR & TIM_SR_UIF)
	{
		/* Clear the interrupt flag */
		TIM3->SR &= ~(TIM_SR_UIF);
	}
}

void SysTick_Handler( void ) {
	systick = systick + 1;
}


/* This DMA interrupt handler is used by the ADC to copy data to a buffer */
void DMA1_Channel1_IRQHandler(void)
{
	/* Half way through buffer interrupt */
	if (DMA1->ISR & DMA_ISR_HTIF1)
	{
		/* Clear the interrupt flag */
		DMA1->IFCR = DMA_IFCR_CHTIF1;
	}
	/* End of buffer interrupt */
	else if (DMA1->ISR & DMA_ISR_TCIF1)
	{
		/* Clear the interrupt flag */
		DMA1->IFCR = DMA_IFCR_CTCIF3;
	}
}

/* This DMA interrupt handler is used by timer 3 for the WS2812 LEDs */
#if defined RGB || defined RGBW
void DMA1_Channel2_3_IRQHandler(void)
{
	/* Half way through buffer interrupt */
	if (DMA1->ISR & DMA_ISR_HTIF3)
	{
		/* Fill the lower half of the buffer */
		led_fill_dma_buffer(&leds, DMA_LOWER_HALF_OFFSET, DMA_HALF_SIZE);

		/* Clear the interrupt flag */
		DMA1->IFCR = DMA_IFCR_CHTIF3;
	}
	/* End of buffer interrupt */
	else if (DMA1->ISR & DMA_ISR_TCIF3)
	{
		/* Disable timer */
		if (leds.arr_pos > (leds.num_leds * leds.led_bytes))
		{
			TIM3->CR1 &= ~(TIM_CR1_CEN);
		}

		/* Fill the upper half of the buffer */
		led_fill_dma_buffer(&leds, DMA_UPPER_HALF_OFFSET, DMA_HALF_SIZE);
	
		/* Clear the interrupt flag */
		DMA1->IFCR = DMA_IFCR_CTCIF3;
	}
}
#endif

void USART1_IRQHandler(void)
{
	/* Check for TX register empty interrupt flag */
	if (USART1->ISR & USART_ISR_TXE)
	{
		/* Output next character in the ringbuffer */
		char temp = ringbuffer_read(USART1_settings.tx_buffer);
		/* Check for an empty buffer, if so disable the transmit */
		if (USART1_settings.tx_buffer->buffer_empty)
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
		ringbuffer_write(USART1_settings.rx_buffer, temp);
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
