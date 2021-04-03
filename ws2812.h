#ifndef WS2812_H
#define WS2812_H

#include "inc/stm32f030x6.h"

#include "peripherals.h"

#include <stdint.h>
#include <stdbool.h>

/* Set the number of leds of data you want to buffer */
/* NB: This should always be an even number! */
#define DMA_LED_BUFF 8
#define DMA_BUFF_SIZE (DMA_LED_BUFF * 8 * 3)
#define DMA_HALF_SIZE (DMA_BUFF_SIZE/2)
#define DMA_LOWER_HALF_OFFSET 0
#define DMA_UPPER_HALF_OFFSET DMA_HALF_SIZE

typedef enum
{
	LED_GRB = 0,
	LED_RGB = 1,
	LED_RGBW = 2,
} LED_FORMAT_E;

typedef struct
{
	uint8_t *data;
	uint8_t bit_mask;
	uint16_t arr_pos;
	const uint16_t num_leds;
	const LED_FORMAT_E data_format;
	const uint8_t led_bytes;
} led_t;

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

void led_breathe_effect(led_t *leds, uint8_t max_red, uint8_t max_green,
						uint8_t max_blue, uint8_t steps, uint32_t delay_ms);

void led_pulse(led_t *leds, uint8_t background_red,
				uint8_t background_green, uint8_t background_blue,
				uint8_t pulse_red, uint8_t pulse_green, uint8_t pulse_blue,
				uint32_t pulse_move_speed_ms);

void led_rgbw_pulse(led_t *leds,
					uint8_t background_red,
					uint8_t background_green,
					uint8_t background_blue,
					uint8_t background_white,
					uint8_t pulse_red,
					uint8_t pulse_green,
					uint8_t pulse_blue,
					uint8_t pulse_white,
					uint32_t pulse_move_speed_ms);

void led_rgbw_centre_ripple(led_t *leds,
							uint8_t red,
							uint8_t green,
							uint8_t blue,
							uint8_t white,
							uint32_t ripple_update_delay);

void led_show(led_t *leds, TIM_TypeDef *TIMx);

#endif //WS2812_H
