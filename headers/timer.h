#ifndef TIMER_H
#define TIMER_H
#include "../inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	TIM_CHAN_1 = 0,
	TIM_CHAN_2 = 1,
	TIM_CHAN_3 = 2,
	TIM_CHAN_4 = 3,
} TIMER_CHANNEL_E;

void init_timer(TIM_TypeDef *TIMx);

void start_timer(TIM_TypeDef *TIMx, uint16_t prescale, uint16_t count);

void disable_timer(TIM_TypeDef *TIMx);

void setup_timer_capture_compare(TIM_TypeDef *TIMx, const TIMER_CHANNEL_E channel,
								 uint16_t ARR, uint16_t CCR, uint16_t prescale, bool flip_polarity, bool preload);

#endif // TIMER_H
