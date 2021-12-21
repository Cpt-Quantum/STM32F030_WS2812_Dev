#include "inc/stm32f030x6.h"

#include "circular_buffer.h"
#include "peripherals.h"
#include "gpio.h"
#include "adc.h"
#include "timer.h"
#include "usart.h"
#include "ws2812.h"
#include "avg.h"
#include "string_format.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Set whether the board uses RGB or RGBW leds */
#define RGBW

/* Define pin connections */
#define LED_PIN 0
#define BTN_PIN 1

#define NUM_LEDS 12
#define MAX_BRIGHTNESS 30 //NB: Changed from 180 to avoid blinding me

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
#define AHB_FREQ (SYSCLK_FREQ / 1)
#define APB_FREQ (AHB_FREQ / 1)

/* An ARR value of 2400 gives a sample rate of 20kHz */
#define ADC_CLK_DIV 2400
/* Use a value to give a slower sampling frequency whilst debugging */
//#define ADC_CLK_DIV 1500

/* Message to print over USART every second */
char usart_message[] = "Hello world!\n";

volatile char rx_char_buff = '\0';

/* Register the needed ringbuffers for USART1 RX and TX */
#define USART_BUFFER_LENGTH 100
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
	.prescaler = SYSCLK_FREQ / 115200,
	//.prescaler = SYSCLK_FREQ / 800000,
	.txe_interrupt_en = true,
	.rxne_interrupt_en = true,
	.tc_interrupt_en = false,
	.interrupt_prio = 3,
	.tx_buffer = &usart_tx_buffer,
	.rx_buffer = &usart_rx_buffer};

/* ADC data buffer */
#define ADC_DMA_BUFFER_SIZE 100
#define ADC_BUFFER_SIZE (ADC_DMA_BUFFER_SIZE / 2)
uint16_t adc_dma_buff[ADC_DMA_BUFFER_SIZE];
//uint16_t adc_data[ADC_BUFFER_SIZE];

/* Define objects for the moving average to be performed over 10Hz, this is used for removing */
/* the DC offset from the input signal before power calculations are made. */
#define DETREND_MIN_FREQ 20
#define DETREND_MAX_FREQ 20000
#define DETREND_SAMPLES ((2 * DETREND_MAX_FREQ / DETREND_MIN_FREQ) / ADC_BUFFER_SIZE)
uint16_t detrend_avg_buffer[DETREND_SAMPLES] = {0};
moving_avg_t average_10Hz = {
	.data_buff = detrend_avg_buffer,
	.data_buff_length = DETREND_SAMPLES,
	.current_total = 0,
	.head = 0,
	.tail = 0,
	.first_cycle = true};
/* Define variable for holding the average power at the full sample rate of the ADC */
uint32_t power_avg = 0;
/* Define the number of cycles to average the signal power over */
#define POWER_AVG_CYCLES 100
/* Define the gain & normalisation factor */
#define SIGNAL_GAIN 1
#define SIGNAL_NORMALISATION (50000 / SIGNAL_GAIN)

/* Register callback function for the ADC interrupt */
/* Note that this will be called twice per full buffer cycle, as it's called */
/* for both the half full and full buffer interrupts. */
void adc_callback(ADC_STATUS_E adc_status)
{
	//	gpio_output(GPIOA, PIN_9, 1);
	//	gpio_output(GPIOA, PIN_9, 0);
	//char adc_data_formatted[ADC_BUFFER_SIZE];
	char adc_data_formatted[2 * ADC_BUFFER_SIZE];
	uint32_t avg = 0;
	uint32_t moving_avg = 0;
	int16_t detrend_data_buff;
	uint32_t power = 0;
	static uint8_t adc_interrupt_count = 0;
	if (adc_status == ADC_BUFFER_FULL)
	{
		adc_interrupt_count += 1;
		/* Calculate the average of the input samples */
		avg = average(&adc_dma_buff[ADC_BUFFER_SIZE], ADC_BUFFER_SIZE);
		/* Add this average to the moving average buffer */
		moving_avg = moving_average(&average_10Hz, avg);
		/* Copy top half to buffer */
		for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++)
		{
			/* Use the moving average to detrend the data before adding it to the USART output buffer */
			detrend_data_buff = (int16_t)(adc_dma_buff[i + ADC_BUFFER_SIZE] - moving_avg);
			power += (uint32_t)(detrend_data_buff * detrend_data_buff);
			//adc_data_formatted[(2 * i)] = (char)(detrend_data_buff & 0x00FF);
			//adc_data_formatted[(2 * i) + 1] = (char)((detrend_data_buff & 0xFF00) >> 8);
			//adc_data_formatted[(2 * i)] = (char)(adc_dma_buff[i + ADC_BUFFER_SIZE] & 0x00FF);
			//adc_data_formatted[(2 * i) + 1] = (char)((adc_dma_buff[i + ADC_BUFFER_SIZE] & 0xFF00) >> 8);
		}
	}
	else if (adc_status == ADC_BUFFER_HALF_FULL)
	{
		adc_interrupt_count += 1;
		/* Calculate the average of the input samples */
		avg = average(&adc_dma_buff[0], ADC_BUFFER_SIZE);
		/* Add this average to the moving average buffer */
		moving_avg = moving_average(&average_10Hz, avg);
		/* Copy bottom half to buffer */
		for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++)
		{
			/* Use the moving average to detrend the data before adding it to the USART output buffer */
			detrend_data_buff = (int16_t)(adc_dma_buff[i] - moving_avg);
			power += (uint32_t)(detrend_data_buff * detrend_data_buff);
			//adc_data_formatted[(2 * i)] = (char)(detrend_data_buff & 0x00FF);
			//adc_data_formatted[(2 * i) + 1] = (char)((detrend_data_buff & 0xFF00) >> 8);
			//adc_data_formatted[(2 * i)] = (char)(adc_dma_buff[i] & 0x00FF);
			//adc_data_formatted[(2 * i) + 1] = (char)((adc_dma_buff[i] & 0xFF00) >> 8);
		}
	}
	/* Calculate the average for power over this data set */
	power_avg += power / ADC_BUFFER_SIZE;
	if (adc_interrupt_count >= POWER_AVG_CYCLES)
	{
		/* Rescale the power by the number of cycles */
		power_avg = power_avg / POWER_AVG_CYCLES;

		/* Send out the power_avg value over USART for debugging */
		char power_formatted[10];
		uint32_to_string(power_formatted, power_avg);
		//usart_write_tx_buffer(USART1_settings, power_formatted,
		//					  sizeof(power_formatted) / sizeof(power_formatted[0]));
		//usart_write_tx_buffer(USART1_settings, "\n\r", 2);
		//usart_start_tx(USART1_settings);

		/* Call LED update routine */
		led_rgbw_intensity(&leds, power_avg, SIGNAL_NORMALISATION);
		//led_rgbw_adc_intensity(&leds, 20, SIGNAL_NORMALISATION);

		/* Reset variables */
		power_avg = 0;
		adc_interrupt_count = 0;
	}
	//usart_write_tx_buffer(USART1_settings, adc_data_formatted, 2 * ADC_BUFFER_SIZE);
	//usart_start_tx(USART1_settings);
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
	clock_setup(false, true, PLL_MULT_X12, PPRE_DIV_1, HPRE_DIV_1);

#if defined RGB || defined RGBW
	/* Initialise the timer */
	init_timer(TIM3);
#endif

	/* Turn the LED on */
	gpio_init(GPIOA, LED_PIN, GPIO_OUTPUT, GPIO_AF0, GPIO_LOW_SPEED);
	gpio_output(GPIOA, LED_PIN, 1);

	/* Set up the input button */
	gpio_init(GPIOA, BTN_PIN, GPIO_INPUT, GPIO_AF0, GPIO_LOW_SPEED);

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
		//		/* Set all LEDs to red and send out the data */
		//		led_rgbw_write_all(&leds, MAX_BRIGHTNESS, 0, 0, 0);
		//		led_show(&leds, TIM3);
		//
		//		/* Wait 1s */
		//		delay_ms(1000);
		//
		//		/* Set all LEDs to green and send out the data */
		//		led_rgbw_write_all(&leds, 0, MAX_BRIGHTNESS, 0, 0);
		//		led_show(&leds, TIM3);
		//
		//		/* Wait 1s */
		//		delay_ms(1000);
		//
		//		/* Set all LEDs to blue and send out the data */
		//		led_rgbw_write_all(&leds, 0, 0, MAX_BRIGHTNESS, 0);
		//		led_show(&leds, TIM3);
		//
		//		/* Wait 1s */
		//		delay_ms(1000);
		//
		//		/* Set all LEDs to white and send out the data */
		//		led_rgbw_write_all(&leds, 0, 0, 0, MAX_BRIGHTNESS);
		//		led_show(&leds, TIM3);
		//
		//		/* Wait 1s */
		//		delay_ms(1000);

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