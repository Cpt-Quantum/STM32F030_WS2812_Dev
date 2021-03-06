#include "../inc/stm32f030x6.h"

#include "ws2812.h"
#include "../headers/peripherals.h"
#include "../headers/timer.h"

#include <stdint.h>
#include <stdbool.h>

#define DATA_1_HIGH 38
#define DATA_1_LOW 22
#define DATA_1_PERIOD (DATA_1_HIGH + DATA_1_LOW)
#define DATA_0_HIGH 19
#define DATA_0_LOW 41
#define DATA_0_PERIOD (DATA_0_HIGH + DATA_0_LOW)

/* Defaults for LED effects */
#define LED_BREATHE_STEPS_DEFAULT 12
#define LED_MAX_SPEED LED_SPEED_8

/* This buffer is used to store capture/compare values for the timer */
uint16_t CCR_buffer[DMA_BUFF_SIZE];

void led_fill_dma_buffer(led_t *leds, uint16_t offset, uint16_t length)
{
	for (uint16_t i = offset; i < (length + offset); i++)
	{
		/* Add padding if the array position exceeds the number of LEDs */
		if (leds->arr_pos >= (leds->num_leds * leds->led_bytes))
		{
			CCR_buffer[i] = 0;
		}
		else
		{
			if (leds->data[leds->arr_pos] & leds->bit_mask)
			{
				CCR_buffer[i] = DATA_1_HIGH;
			}
			else
			{
				CCR_buffer[i] = DATA_0_HIGH;
			}
		}
		if (leds->bit_mask == 1)
		{
			leds->bit_mask = 0B10000000;
			leds->arr_pos = leds->arr_pos + 1;
		}
		else
		{
			leds->bit_mask = leds->bit_mask >> 1;
		}
	}
}

void dma_setup(void)
{
	/* Enable the clock for DMA channel 3 */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	/* First make sure DMA is disabled */
	DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
	/* Enable the DMA transfer on timer 3 */
	TIM3->DIER |= TIM_DIER_CC4DE;
	/* Configure the peripheral data register address */
	DMA1_Channel3->CPAR = (uint32_t)&TIM3->CCR4;
	/* Configure the memory address */
	DMA1_Channel3->CMAR = (uint32_t)&CCR_buffer[0];
	/* Setup the DMA configuration register */
	/* Set the memory and peripheral size to 16 bit (01) in respective bits */
	/* Set priority to very high (11) in the PL bits */
	/* Set the memory increment to true, whilst the peripheral increment is false */
	/* Set the DMA to circular mode */
	/* Set the direction to transfer from memory to peripheral */
	/* Enable the half transfer and transfer complete interrupts */
	DMA1_Channel3->CCR = (uint32_t)0;
	DMA1_Channel3->CCR |= (DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
						   DMA_CCR_PL_0 | DMA_CCR_PL_1 | DMA_CCR_MINC |
						   DMA_CCR_CIRC | DMA_CCR_DIR |
						   DMA_CCR_HTIE | DMA_CCR_TCIE);
	/* Set the size of the DMA transfer to the buffer size */
	DMA1_Channel3->CNDTR = DMA_BUFF_SIZE;
	/* Set the priority to high for DMA */
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
	/* Enable DMA interrupts for channel 3 in the NVIC */
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	/* Enable DMA channel 3 */
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void led_rgb_write_all(led_t *leds, uint8_t red, uint8_t green, uint8_t blue)
{
	for (uint16_t i = 0; i < leds->num_leds; i++)
	{
		leds->data[(3 * i)] = green;
		leds->data[(3 * i) + 1] = red;
		leds->data[(3 * i) + 2] = blue;
	}
}

void led_rgb_write_pixel(led_t *leds, uint16_t pixel, uint8_t red,
						 uint8_t green, uint8_t blue)
{
	leds->data[(3 * pixel)] = green;
	leds->data[(3 * pixel) + 1] = red;
	leds->data[(3 * pixel) + 2] = blue;
}

void led_rgbw_write_all(led_t *leds, uint8_t red, uint8_t green, uint8_t blue,
						uint8_t white)
{
	for (uint16_t i = 0; i < leds->num_leds; i++)
	{
		leds->data[(4 * i)] = green;
		leds->data[(4 * i) + 1] = red;
		leds->data[(4 * i) + 2] = blue;
		leds->data[(4 * i) + 3] = white;
	}
}

void led_rgbw_write_pixel(led_t *leds, uint16_t pixel, uint8_t red,
						  uint8_t green, uint8_t blue, uint8_t white)
{
	leds->data[(4 * pixel)] = green;
	leds->data[(4 * pixel) + 1] = red;
	leds->data[(4 * pixel) + 2] = blue;
	leds->data[(4 * pixel) + 3] = white;
}

void led_rgb_breathe_effect(led_t *leds, led_effect_t *effect, uint8_t max_red, uint8_t max_green,
							uint8_t max_blue, uint8_t steps, bool *first_cycle)
{
	/* Declare looping variables */
	static uint8_t j = 0;
	/* Declare variable for tracking progress of function */
	static bool increasing_brightness = true;

	/* Check for a first cycle to initialise the counting and state variables */
	if (*first_cycle)
	{
		j = steps;
		increasing_brightness = true;
		*first_cycle = false;
	}

	/* Calculate and set the LED effect->brightnesses for all LEDs */
	for (uint8_t i = 0; i < leds->num_leds; i++)
	{
		leds->data[(3 * i)] = max_green / j;
		leds->data[(3 * i) + 1] = max_red / j;
		leds->data[(3 * i) + 2] = max_blue / j;
	}

	/* Update j depending on the current state, also checked if j has reached the end of */
	/* its current state and update the state variable accordingly. */
	if (increasing_brightness == true)
	{
		j--;
		if (j == 1)
		{
			increasing_brightness = false;
		}
	}
	else if (increasing_brightness == false)
	{
		j++;
		if (j == steps * ((LED_MAX_SPEED - effect->effect_speed) + 2))
		{
			increasing_brightness = true;
		}
	}
}

void led_rgb_pulse(led_t *leds, led_effect_t *effect, uint8_t background_red,
				   uint8_t background_green, uint8_t background_blue,
				   uint8_t pulse_red, uint8_t pulse_green, uint8_t pulse_blue,
				   bool *first_cycle)
{
	/* Declare the counting variable */
	static uint16_t i = 1;
	/* Declare a frame counter to be able to set the speed of the effect */
	static uint8_t frame_counter = 0;

	if (*first_cycle)
	{
		/* Initialise background_colour */
		led_rgb_write_all(leds, background_red, background_green, background_blue);
		i = 1;

		/* Set the frame counter to 0 to switch to this effect immediately */
		frame_counter = 0;

		*first_cycle = false;
	}

	/* Check if the frame counter has finished, if so update the LEDs */
	if (frame_counter == 0)
	{
		/* Update the counting variable and check if the effect has gone through a whole cycle */
		i++;
		if (i >= leds->num_leds)
		{
			/* Initialise background_colour */
			led_rgb_write_all(leds, background_red, background_green, background_blue);
			i = 1;
		}
		frame_counter = LED_MAX_SPEED - effect->effect_speed;
	}

	/* Reset the previous pixel to the background colour */
	led_rgb_write_pixel(leds, (i - 1), background_red, background_green, background_blue);
	/* Set the current pixel to the foreground colour */
	led_rgb_write_pixel(leds, i, pulse_red, pulse_green, pulse_blue);

	/* Finally decrement the frame counter */
	frame_counter--;
}

void led_rgbw_breathe_effect(led_t *leds, led_effect_t *effect, uint8_t max_red, uint8_t max_green,
							 uint8_t max_blue, uint8_t max_white, uint8_t steps, bool *first_cycle)
{
	/* Declare looping variables */
	static uint8_t j = 0;
	/* Declare variable for tracking progress of function */
	static bool increasing_brightness = true;

	/* Check for a first cycle to initialise the counting and state variables */
	if (*first_cycle)
	{
		j = steps;
		increasing_brightness = true;
		*first_cycle = false;
	}

	/* Calculate and set the LED effect->brightnesses for all LEDs */
	for (uint8_t i = 0; i < leds->num_leds; i++)
	{
		leds->data[(4 * i)] = max_green / j;
		leds->data[(4 * i) + 1] = max_red / j;
		leds->data[(4 * i) + 2] = max_blue / j;
		leds->data[(4 * i) + 3] = max_white / j;
	}

	/* Update j depending on the current state, also checked if j has reached the end of */
	/* its current state and update the state variable accordingly. */
	if (increasing_brightness == true)
	{
		j--;
		if (j == 1)
		{
			increasing_brightness = false;
		}
	}
	else if (increasing_brightness == false)
	{
		j++;
		/* Note that the minimum number of steps (max speed) is 2 * input_steps */
		if (j == steps * ((LED_MAX_SPEED - effect->effect_speed) + 2))
		{
			increasing_brightness = true;
		}
	}
}

void led_rgbw_pulse(led_t *leds, led_effect_t *effect,
					uint8_t background_red,
					uint8_t background_green,
					uint8_t background_blue,
					uint8_t background_white,
					uint8_t pulse_red,
					uint8_t pulse_green,
					uint8_t pulse_blue,
					uint8_t pulse_white,
					bool *first_cycle)
{
	/* Declare the counting variable */
	static uint16_t i = 1;
	/* Declare a frame counter to be able to set the speed of the effect */
	static uint8_t frame_counter = 0;

	if (*first_cycle)
	{
		/* Initialise background_colour */
		led_rgbw_write_all(leds, background_red, background_green, background_blue, background_white);
		i = 1;

		/* Set the frame counter to 0 to switch to this effect immediately */
		frame_counter = 0;

		*first_cycle = false;
	}

	/* Check if the frame counter has finished, if so update the LEDs */
	if (frame_counter == 0)
	{
		/* Update the counting variable and check if the effect has gone through a whole cycle */
		i++;
		if (i >= leds->num_leds)
		{
			/* Initialise background_colour */
			led_rgbw_write_all(leds, background_red, background_green, background_blue, background_white);
			i = 1;
		}
		frame_counter = LED_MAX_SPEED - effect->effect_speed;
	}

	/* Reset the previous pixel to the background colour */
	led_rgbw_write_pixel(leds, (i - 1), background_red, background_green, background_blue, background_white);
	/* Set the current pixel to the foreground colour */
	led_rgbw_write_pixel(leds, i, pulse_red, pulse_green, pulse_blue, pulse_white);

	/* Finally decrement the frame counter */
	frame_counter--;
}

void led_rgbw_centre_ripple(led_t *leds,
							uint8_t red,
							uint8_t green,
							uint8_t blue,
							uint8_t white,
							uint32_t ripple_update_delay)
{
	/* Calculate the centre positions */
	uint16_t centre_r = (leds->num_leds + 1) / 2;
	uint16_t centre_l = centre_r - 1;

	/* Set the two middle pixels to the default colour and then add the */
	/* colour to the next adjacent pixels. Continue until all LEDs are */
	/* illuminated. */
	for (uint16_t i = 0; i < ((leds->num_leds) / 2); i++)
	{
		led_rgbw_write_pixel(leds, centre_l - i,
							 red,
							 green,
							 blue,
							 white);
		led_rgbw_write_pixel(leds, centre_r + i,
							 red,
							 green,
							 blue,
							 white);
		led_show(leds);

		delay_ms(ripple_update_delay);
	}

	/* Now remove the coloured pixels one by one back to the central pixels */
	for (uint16_t i = (((leds->num_leds) / 2) - 1); i > 0; i--)
	{
		led_rgbw_write_pixel(leds, centre_l - i, 0, 0, 0, 0);
		led_rgbw_write_pixel(leds, centre_r + i, 0, 0, 0, 0);
		led_show(leds);

		delay_ms(ripple_update_delay);
	}
}

void led_rgbw_intensity(led_t *leds, uint32_t adc_val, uint32_t normalisation)
{
	/* Need to scale the intensity by the total number of LEDs */
	uint16_t intensity = (uint16_t)((adc_val * leds->num_leds) / normalisation);

	/* Catch for an intensity value clipping the number of LEDs */
	if (intensity > leds->num_leds)
	{
		intensity = leds->num_leds;
	}

	/* Now turn on the number of LEDs according to the intensity value */
	for (uint16_t i = 0; i < intensity; i++)
	{
		if (i < (leds->num_leds / 4))
		{
			led_rgbw_write_pixel(leds, i, 5, 5, 0, 0);
		}
		else if (i < ((3 * leds->num_leds) / 4))
		{
			led_rgbw_write_pixel(leds, i, 0, 10, 0, 0);
		}
		else
		{
			led_rgbw_write_pixel(leds, i, 10, 0, 0, 0);
		}
	}

	/* Ensure that any pixels higher than the intensity are off */
	for (uint16_t i = intensity; i < leds->num_leds; i++)
	{
		led_rgbw_write_pixel(leds, i, 0, 0, 0, 0);
	}
}

void led_init(void)
{
	/* Initialise count to a high enough value such that the output is held */
	/* low before data transmission starts. */
	TIM3->CNT = 80;

	/* Setup the timer for capture/compare mode */
	setup_timer_capture_compare(TIM3, TIM_CHAN_4, DATA_1_PERIOD, 0, 0, false, true);
}

void led_update_effect(led_t *leds, led_effect_t *effect, bool *effect_first_cycle)
{
	/* Check the format of the LEDs */
	switch (leds->data_format)
	{
	case LED_GRB:
		switch (effect->current_effect)
		{
		case LED_OFF:
			led_rgb_write_all(leds, 0, 0, 0);
			break;
		case LED_PULSE_RED:
			led_rgb_pulse(leds, effect,
						  effect->brightness, 0, 0,
						  (effect->brightness * 2), 0, 0,
						  effect_first_cycle);
			break;
		case LED_PULSE_GREEN:
			led_rgb_pulse(leds, effect,
						  0, effect->brightness, 0,
						  0, (effect->brightness * 2), 0,
						  effect_first_cycle);
			break;
		case LED_PULSE_BLUE:
			led_rgb_pulse(leds, effect,
						  0, 0, effect->brightness,
						  0, 0, (effect->brightness * 2),
						  effect_first_cycle);
			break;
		case LED_PULSE_WHITE:
			led_rgb_pulse(leds, effect,
						  effect->brightness / 3, effect->brightness / 3, effect->brightness / 3,
						  (effect->brightness * 2) / 3, (effect->brightness * 2) / 3, (effect->brightness * 2) / 3,
						  effect_first_cycle);
			break;
		case LED_PULSE_ORANGE:
			led_rgb_pulse(leds, effect,
						  effect->brightness / 2, effect->brightness / 2, 0,
						  (effect->brightness * 2) / 2, (effect->brightness * 2) / 2, 0,
						  effect_first_cycle);
			break;
		case LED_PULSE_PURPLE:
			led_rgb_pulse(leds, effect,
						  effect->brightness / 2, 0, effect->brightness / 2,
						  (effect->brightness * 2) / 2, 0, (effect->brightness * 2) / 2,
						  effect_first_cycle);
			break;
		case LED_PULSE_YELLOW:
			led_rgb_pulse(leds, effect,
						  0, effect->brightness / 2, effect->brightness / 2,
						  0, (effect->brightness * 2) / 2, (effect->brightness * 2) / 2,
						  effect_first_cycle);
			break;
		case LED_PULSE_CUSTOM:
			led_rgb_pulse(leds, effect,
						  effect->custom_input_1.rgb_custom_colour.red,
						  effect->custom_input_1.rgb_custom_colour.green,
						  effect->custom_input_1.rgb_custom_colour.blue,
						  effect->custom_input_2.rgb_custom_colour.red,
						  effect->custom_input_2.rgb_custom_colour.green,
						  effect->custom_input_2.rgb_custom_colour.blue,
						  effect_first_cycle);
			break;
		case LED_BREATHE_RED:
			led_rgb_breathe_effect(leds, effect,
								   effect->brightness, 0, 0,
								   LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_GREEN:
			led_rgb_breathe_effect(leds, effect,
								   0, effect->brightness, 0,
								   LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_BLUE:
			led_rgb_breathe_effect(leds, effect,
								   0, 0, effect->brightness,
								   LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_WHITE:
			led_rgb_breathe_effect(leds, effect,
								   effect->brightness / 3, effect->brightness / 3, effect->brightness / 3,
								   LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_ORANGE:
			led_rgb_breathe_effect(leds, effect,
								   effect->brightness / 2, effect->brightness / 2, 0,
								   LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_PURPLE:
			led_rgb_breathe_effect(leds, effect,
								   effect->brightness / 2, 0, effect->brightness / 2,
								   LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_YELLOW:
			led_rgb_breathe_effect(leds, effect,
								   0, effect->brightness / 2, effect->brightness / 2,
								   LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_CUSTOM:
			led_rgb_breathe_effect(leds, effect,
								   effect->custom_input_1.rgb_custom_colour.red,
								   effect->custom_input_1.rgb_custom_colour.green,
								   effect->custom_input_1.rgb_custom_colour.blue,
								   LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_STATIC_RED:
			led_rgb_write_all(leds,
							  effect->brightness, 0, 0);
			break;
		case LED_STATIC_GREEN:
			led_rgb_write_all(leds,
							  0, effect->brightness, 0);
			break;
		case LED_STATIC_BLUE:
			led_rgb_write_all(leds,
							  0, 0, effect->brightness);
			break;
		case LED_STATIC_WHITE:
			led_rgb_write_all(leds,
							  effect->brightness / 3, effect->brightness / 3, effect->brightness / 3);
			break;
		case LED_STATIC_ORANGE:
			led_rgb_write_all(leds,
							  effect->brightness / 2, effect->brightness / 2, 0);
			break;
		case LED_STATIC_PURPLE:
			led_rgb_write_all(leds,
							  effect->brightness / 2, 0, effect->brightness / 2);
			break;
		case LED_STATIC_YELLOW:
			led_rgb_write_all(leds,
							  0, effect->brightness / 2, effect->brightness / 2);
			break;
		case LED_STATIC_CUSTOM:
			led_rgb_write_all(leds,
							  effect->custom_input_1.rgb_custom_colour.red,
							  effect->custom_input_1.rgb_custom_colour.green,
							  effect->custom_input_1.rgb_custom_colour.blue);
			break;
		default:
			break;
		}
		break;
	case LED_RGBW:
		switch (effect->current_effect)
		{
		case LED_OFF:
			led_rgbw_write_all(leds, 0, 0, 0, 0);
			break;
		case LED_PULSE_RED:
			led_rgbw_pulse(leds, effect,
						   effect->brightness, 0, 0, 0,
						   (effect->brightness * 2), 0, 0, 0,
						   effect_first_cycle);
			break;
		case LED_PULSE_GREEN:
			led_rgbw_pulse(leds, effect,
						   0, effect->brightness, 0, 0,
						   0, (effect->brightness * 2), 0, 0,
						   effect_first_cycle);
			break;
		case LED_PULSE_BLUE:
			led_rgbw_pulse(leds, effect,
						   0, 0, effect->brightness, 0,
						   0, 0, (effect->brightness * 2), 0,
						   effect_first_cycle);
			break;
		case LED_PULSE_WHITE:
			led_rgbw_pulse(leds, effect,
						   0, 0, 0, effect->brightness,
						   0, 0, 0, effect->brightness * 2,
						   effect_first_cycle);
			break;
		case LED_PULSE_ORANGE:
			led_rgbw_pulse(leds, effect,
						   effect->brightness / 2, effect->brightness / 2, 0, 0,
						   (effect->brightness * 2) / 2, (effect->brightness * 2) / 2, 0, 0,
						   effect_first_cycle);

			break;
		case LED_PULSE_PURPLE:
			led_rgbw_pulse(leds, effect,
						   effect->brightness / 2, 0, effect->brightness / 2, 0,
						   (effect->brightness * 2) / 2, 0, (effect->brightness * 2) / 2, 0,
						   effect_first_cycle);
			break;
		case LED_PULSE_YELLOW:
			led_rgbw_pulse(leds, effect,
						   0, effect->brightness / 2, effect->brightness / 2, 0,
						   0, (effect->brightness * 2) / 2, (effect->brightness * 2) / 2, 0,
						   effect_first_cycle);
			break;
		case LED_PULSE_CUSTOM:
			led_rgbw_pulse(leds, effect,
						   effect->custom_input_1.rgbw_custom_colour.red,
						   effect->custom_input_1.rgbw_custom_colour.green,
						   effect->custom_input_1.rgbw_custom_colour.blue,
						   effect->custom_input_1.rgbw_custom_colour.white,
						   effect->custom_input_2.rgbw_custom_colour.red,
						   effect->custom_input_2.rgbw_custom_colour.green,
						   effect->custom_input_2.rgbw_custom_colour.blue,
						   effect->custom_input_2.rgbw_custom_colour.white,
						   effect_first_cycle);
			break;
		case LED_BREATHE_RED:
			led_rgbw_breathe_effect(leds, effect,
									effect->brightness, 0, 0, 0,
									LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_GREEN:
			led_rgbw_breathe_effect(leds, effect,
									0, effect->brightness, 0, 0,
									LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_BLUE:
			led_rgbw_breathe_effect(leds, effect,
									0, 0, effect->brightness, 0,
									LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_WHITE:
			led_rgbw_breathe_effect(leds, effect,
									0, 0, 0, effect->brightness,
									LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_ORANGE:
			led_rgbw_breathe_effect(leds, effect,
									effect->brightness / 2, effect->brightness / 2, 0, 0,
									LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_PURPLE:
			led_rgbw_breathe_effect(leds, effect,
									effect->brightness / 2, 0, effect->brightness / 2, 0,
									LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_YELLOW:
			led_rgbw_breathe_effect(leds, effect,
									0, effect->brightness / 2, effect->brightness / 2, 0,
									LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_BREATHE_CUSTOM:
			led_rgbw_breathe_effect(leds, effect,
									effect->custom_input_1.rgbw_custom_colour.red,
									effect->custom_input_1.rgbw_custom_colour.green,
									effect->custom_input_1.rgbw_custom_colour.blue,
									effect->custom_input_1.rgbw_custom_colour.white,
									LED_BREATHE_STEPS_DEFAULT, effect_first_cycle);
			break;
		case LED_STATIC_RED:
			led_rgbw_write_all(leds,
							   effect->brightness, 0, 0, 0);
			break;
		case LED_STATIC_GREEN:
			led_rgbw_write_all(leds,
							   0, effect->brightness, 0, 0);
			break;
		case LED_STATIC_BLUE:
			led_rgbw_write_all(leds,
							   0, 0, effect->brightness, 0);
			break;
		case LED_STATIC_WHITE:
			led_rgbw_write_all(leds,
							   0, 0, 0, effect->brightness);
			break;
		case LED_STATIC_ORANGE:
			led_rgbw_write_all(leds,
							   effect->brightness / 2, effect->brightness / 2, 0, 0);
			break;
		case LED_STATIC_PURPLE:
			led_rgbw_write_all(leds,
							   effect->brightness / 2, 0, effect->brightness / 2, 0);
			break;
		case LED_STATIC_YELLOW:
			led_rgbw_write_all(leds,
							   0, effect->brightness / 2, effect->brightness / 2, 0);
			break;
		case LED_STATIC_CUSTOM:
			led_rgbw_write_all(leds,
							   effect->custom_input_1.rgbw_custom_colour.red,
							   effect->custom_input_1.rgbw_custom_colour.green,
							   effect->custom_input_1.rgbw_custom_colour.blue,
							   effect->custom_input_1.rgbw_custom_colour.white);
			break;
		default:
			break;
		}
	}
}

static inline bool effect_is_static(LED_EFFECT_E effect)
{
	switch (effect)
	{
	case LED_OFF:
		return true;
	case LED_STATIC_RED:
		return true;
	case LED_STATIC_GREEN:
		return true;
	case LED_STATIC_BLUE:
		return true;
	case LED_STATIC_WHITE:
		return true;
	case LED_STATIC_ORANGE:
		return true;
	case LED_STATIC_PURPLE:
		return true;
	case LED_STATIC_YELLOW:
		return true;
	case LED_STATIC_CUSTOM:
		return true;
	default:
		return false;
	}
}

void led_update_frame_data(led_t *leds, led_effect_t *effect)
{
	if (effect->effect_updated && effect->num_blacklisted_effects > 0)
	{
		/* Check if effect is blacklisted, only need to do this when effect changes */
		/* If it is, increment the effect to the next available effect. Continue until */
		/* the current effect has been updated equal to the number of effects that are */
		/* blacklisted. Check that the effect hasn't reached the last entry, if so, wrap .*/
		for (uint8_t i = 0; i < effect->num_blacklisted_effects; i++)
		{
			for (uint8_t j = 0; j < effect->num_blacklisted_effects; j++)
			{
				if (effect->current_effect == effect->blacklisted_effects[j])
				{
					effect->current_effect++;
					if (effect->current_effect >= LED_EFFECT_RESERVED)
					{
						effect->current_effect = 0;
					}
					i++;
				}
			}
		}
	}

	/* Check if the effect is a static effect, if so return as nothing needs to be done */
	if (effect_is_static(effect->current_effect) && !effect->effect_updated)
	{
		return;
	}

	led_update_effect(leds, effect, &effect->effect_updated);
	led_show(leds);
}

void led_show(led_t *leds)
{
	/* Reset the array position and bit mask before starting */
	leds->bit_mask = 0B10000000;
	leds->arr_pos = 0;

	/* Reset the DMA */
	DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
	DMA1_Channel3->CCR |= DMA_CCR_EN;

	/* Fill full buffer before starting */
	led_fill_dma_buffer(leds, DMA_LOWER_HALF_OFFSET, DMA_BUFF_SIZE);

	dma_setup();

	leds->TIMx->CCER |= TIM_CCER_CC4E;
	leds->TIMx->CR1 |= TIM_CR1_CEN;
}
