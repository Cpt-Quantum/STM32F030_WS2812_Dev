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
#include <stddef.h>

/* Set whether the board uses RGB or RGBW leds */
//#define RGBW

/* Define pin connections */
#define LED_PIN 0

#define NUM_LEDS 12
#define MAX_BRIGHTNESS 10 //NB: Changed from 180 to avoid blinding me

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
#define AHB_FREQ (SYSCLK_FREQ / 4)
#define APB_FREQ (AHB_FREQ / 1)

/* An ARR value of 1200 gives a sample rate of 10kHz */
#define ADC_CLK_DIV 600
/* Use a value to give 8kHz whilst debugging */
//#define ADC_CLK_DIV 1500

/* Message to print over USART every second */
char usart_message[] = "Hello world!\n";

volatile char rx_char_buff = '\0';

/* Register the needed ringbuffers for USART1 RX and TX */
#define USART_BUFFER_LENGTH 128
volatile char usart_tx_buf[USART_BUFFER_LENGTH];
volatile char usart_rx_buf[USART_BUFFER_LENGTH];
ringbuffer_t usart_tx_buffer = {
	.buffer = usart_tx_buf,
	.head = 0,
	.tail = 0,
	.length = sizeof(usart_tx_buf),
	.buffer_empty = true,
	.buffer_full = false};
ringbuffer_t usart_rx_buffer = {
	.buffer = usart_rx_buf,
	.head = 0,
	.tail = 0,
	.length = sizeof(usart_rx_buf),
	.buffer_empty = true,
	.buffer_full = false};

/* Initialise the struct for USART1 settings */
USART_t USART1_settings = {
	.USARTx = USART1,
	.clk_src = USART_CLK_SYSCLK,
	.prescaler = SYSCLK_FREQ / 800000,
	.txe_interrupt_en = true,
	.rxne_interrupt_en = true,
	.tc_interrupt_en = false,
	.interrupt_prio = 3,
	.tx_buffer = &usart_tx_buffer,
	.rx_buffer = &usart_rx_buffer};

/* ADC data buffer */
#define ADC_DMA_BUFFER_SIZE 128
#define ADC_BUFFER_SIZE (ADC_DMA_BUFFER_SIZE / 2)
uint16_t adc_dma_buff[ADC_DMA_BUFFER_SIZE];
//uint16_t adc_data[ADC_BUFFER_SIZE];

/* Register callback function for the ADC interrupt */
/* Note that this will be called twice per full buffer cycle, as it's called */
/* for both the half full and full buffer interrupts. */
void adc_callback(ADC_STATUS_E adc_status)
{
	//	gpio_output(GPIOA, PIN_9, 1);
	//	gpio_output(GPIOA, PIN_9, 0);
	//char adc_data_formatted[ADC_BUFFER_SIZE];
	char adc_data_formatted[2 * ADC_BUFFER_SIZE];
	if (adc_status == ADC_BUFFER_FULL)
	{
		/* Copy top half to buffer */
		for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++)
		{
			//adc_data_formatted[i] = (char)((adc_dma_buff[i + ADC_BUFFER_SIZE] & 0x0FF0) >> 4);
			adc_data_formatted[(2 * i)] = (char)(adc_dma_buff[i + ADC_BUFFER_SIZE] & 0x00FF);
			adc_data_formatted[(2 * i) + 1] = (char)((adc_dma_buff[i + ADC_BUFFER_SIZE] & 0xFF00) >> 8);
		}
	}
	else if (adc_status == ADC_BUFFER_HALF_FULL)
	{
		/* Copy bottom half to buffer */
		for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++)
		{
			//adc_data_formatted[i] = (char)((adc_dma_buff[i] & 0x0FF0) >> 4);
			adc_data_formatted[(2 * i)] = (char)(adc_dma_buff[i] & 0x00FF);
			adc_data_formatted[(2 * i) + 1] = (char)((adc_dma_buff[i] & 0xFF00) >> 8);
		}
	}
	//usart_write_tx_buffer(USART1_settings, adc_data_formatted, ADC_BUFFER_SIZE);
	usart_write_tx_buffer(USART1_settings, adc_data_formatted, 2 * ADC_BUFFER_SIZE);
	usart_start_tx(USART1_settings);
}

/* Initialise the struct for the ADC settings */
ADC_t ADC_settings = {
	.channel_select = (1 << 6),
	.data = adc_dma_buff,
	.data_length = ADC_DMA_BUFFER_SIZE,
	.clock_div = ADC_CLK_DIV,
	.callback = &adc_callback};

int main(void)
{
	/* Initialisation */

	/* Setup flash wait cycles */
	FLASH->ACR &= ~(0x00000017);
	FLASH->ACR |= (FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);

	/* Initialise system clock */
	clock_setup(false, true, PLL_MULT_X12, PPRE_DIV_1, HPRE_DIV_4);

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
	adc_init(&ADC_settings);

	/* Enable PB1 with alternate functionality */
	gpio_init(GPIOB, PIN_1, GPIO_ALT_MODE, GPIO_AF1, GPIO_HIGH_SPEED);

	/* Enable USART1 */
	usart_init(&USART1_settings);

	/* Setup the SysTick peripheral for 1ms ticks */
	SysTick_Config(AHB_FREQ / 1000);

	/* Initialise PA9 and PA10 as output for debug testing */
	gpio_init(GPIOA, PIN_9, GPIO_OUTPUT, GPIO_AF0, GPIO_HIGH_SPEED);
	gpio_init(GPIOA, PIN_10, GPIO_OUTPUT, GPIO_AF0, GPIO_HIGH_SPEED);

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
	while (1)
	{
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

void SysTick_Handler(void)
{
	systick = systick + 1;
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