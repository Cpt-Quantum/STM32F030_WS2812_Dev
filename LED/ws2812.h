#ifndef WS2812_H
#define WS2812_H

#include "../inc/stm32f030x6.h"

#include "../headers/peripherals.h"

#include <stdint.h>
#include <stdbool.h>

/* Set the number of leds of data you want to buffer */
/* NB: This should always be an even number! */
#define DMA_LED_BUFF 8
#define DMA_BUFF_SIZE (DMA_LED_BUFF * 8 * 3)
#define DMA_HALF_SIZE (DMA_BUFF_SIZE / 2)
#define DMA_LOWER_HALF_OFFSET 0
#define DMA_UPPER_HALF_OFFSET DMA_HALF_SIZE

typedef struct __attribute__((packed))
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} rgb_t;

typedef struct __attribute__((packed))
{
	uint8_t green;
	uint8_t red;
	uint8_t blue;
} grb_t;

typedef struct __attribute__((packed))
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t white;
} rgbw_t;

typedef enum
{
	LED_OFF = 0,
	LED_PULSE_RED = 1,
	LED_PULSE_GREEN = 2,
	LED_PULSE_BLUE = 3,
	LED_PULSE_WHITE = 4,
	LED_PULSE_ORANGE = 5,
	LED_PULSE_PURPLE = 6,
	LED_PULSE_YELLOW = 7,
	LED_PULSE_CUSTOM = 8,
	LED_BREATHE_RED = 9,
	LED_BREATHE_GREEN = 10,
	LED_BREATHE_BLUE = 11,
	LED_BREATHE_WHITE = 12,
	LED_BREATHE_ORANGE = 13,
	LED_BREATHE_PURPLE = 14,
	LED_BREATHE_YELLOW = 15,
	LED_BREATHE_CUSTOM = 16,
	LED_STATIC_RED = 17,
	LED_STATIC_GREEN = 18,
	LED_STATIC_BLUE = 19,
	LED_STATIC_WHITE = 20,
	LED_STATIC_ORANGE = 21,
	LED_STATIC_PURPLE = 22,
	LED_STATIC_YELLOW = 23,
	LED_STATIC_CUSTOM = 24,
	LED_EFFECT_RESERVED = 25,
} LED_EFFECT_E;

typedef enum
{
	LED_GRB = 0,
	LED_RGB = 1,
	LED_RGBW = 2,
} LED_FORMAT_E;

typedef union
{
	rgb_t rgb_custom_colour;
	rgbw_t rgbw_custom_colour;
	uint32_t custom_value;
} led_custom_input_t;

typedef enum
{
	LED_SPEED_1 = 1,
	LED_SPEED_2 = 2,
	LED_SPEED_3 = 3,
	LED_SPEED_4 = 4,
	LED_SPEED_5 = 5,
	LED_SPEED_6 = 6,
	LED_SPEED_7 = 7,
	LED_SPEED_8 = 8,
} LED_SPEED_E;

typedef struct
{
	uint8_t *data;
	uint8_t bit_mask;
	uint16_t arr_pos;
	const uint16_t num_leds;
	const LED_FORMAT_E data_format;
	const uint8_t led_bytes;
	TIM_TypeDef *TIMx;
} led_t;

typedef struct
{
	LED_EFFECT_E current_effect;
	bool effect_updated;
	LED_SPEED_E effect_speed;
	uint8_t *blacklisted_effects;
	uint8_t num_blacklisted_effects;
	uint8_t brightness;
	led_custom_input_t custom_input_1;
	led_custom_input_t custom_input_2;
} led_effect_t;

void led_fill_dma_buffer(led_t *leds, uint16_t offset, uint16_t length);

void dma_setup(void);

void led_init(void);

void led_rgb_write_all(led_t *leds, uint8_t red, uint8_t green, uint8_t blue);

void led_rgb_write_pixel(led_t *leds, uint16_t pixel, uint8_t red,
						 uint8_t green, uint8_t blue);

void led_rgbw_write_all(led_t *leds, uint8_t red, uint8_t green, uint8_t blue,
						uint8_t white);

void led_rgbw_write_pixel(led_t *leds, uint16_t pixel, uint8_t red,
						  uint8_t green, uint8_t blue, uint8_t white);

void led_rgb_breathe_effect(led_t *leds, led_effect_t *effect, uint8_t max_red, uint8_t max_green,
							uint8_t max_blue, uint8_t steps, bool *first_cycle);

void led_rgb_pulse(led_t *leds, led_effect_t *effect, uint8_t background_red,
				   uint8_t background_green, uint8_t background_blue,
				   uint8_t pulse_red, uint8_t pulse_green, uint8_t pulse_blue,
				   bool *first_cycle);

void led_rgbw_breathe_effect(led_t *leds, led_effect_t *effect,
							 uint8_t max_red, uint8_t max_green,
							 uint8_t max_blue, uint8_t max_white,
							 uint8_t steps, bool *first_cycle);

void led_rgbw_pulse(led_t *leds, led_effect_t *effect,
					uint8_t background_red,
					uint8_t background_green,
					uint8_t background_blue,
					uint8_t background_white,
					uint8_t pulse_red,
					uint8_t pulse_green,
					uint8_t pulse_blue,
					uint8_t pulse_white,
					bool *first_cycle);

void led_rgbw_centre_ripple(led_t *leds,
							uint8_t red,
							uint8_t green,
							uint8_t blue,
							uint8_t white,
							uint32_t ripple_update_delay);

void led_rgbw_intensity(led_t *leds, uint32_t adc_val, uint32_t normalisation);

void led_update_effect(led_t *leds, led_effect_t *effect, bool *effect_first_cycle);

void led_update_frame_data(led_t *leds, led_effect_t *effect);

void led_show(led_t *leds);

#endif // WS2812_H
