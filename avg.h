#ifndef AVG_H
#define AVG_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	uint16_t *data_buff;
	uint16_t data_buff_length;
	uint32_t current_total;
	uint16_t head;
	uint16_t tail;
	bool first_cycle;
} moving_avg_t;

uint32_t average(uint16_t *data_in, uint16_t data_length);
uint32_t power_average(uint16_t *data_in, uint16_t data_length);
uint32_t moving_average(moving_avg_t *moving_avg_buffer, uint16_t new_samp);

#endif /* AVG_G */