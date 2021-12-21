#include <stdint.h>
#include <stdbool.h>

#include "avg.h"

uint32_t average(uint16_t *data_in, uint16_t data_length)
{
	/* Use an oversized intermediate type to avoid overflows from adding multiple values */
	uint32_t avg = 0;
	for (uint16_t i = 0; i < data_length; i++)
	{
		avg += data_in[i];
	}
	avg = avg / data_length;
	return ((uint16_t)avg);
}

uint32_t power_average(uint16_t *data_in, uint16_t data_length)
{
	/* Use an oversized intermediate type to avoid overflows from adding multiple values */
	uint32_t power = 0;
	for (uint16_t i = 0; i < data_length; i++)
	{
		power += data_in[i] * data_in[i];
	}
	return ((uint16_t)(power / data_length));
}

uint32_t moving_average(moving_avg_t *moving_avg_buffer, uint16_t new_samp)
{
	uint32_t avg = 0;
	if (moving_avg_buffer->first_cycle == true)
	{
		/* Add new sample to the sample buffer */
		moving_avg_buffer->data_buff[moving_avg_buffer->head] = new_samp;
		(moving_avg_buffer->head)++;

		/* Compute the moving average and add it to the struct */
		moving_avg_buffer->current_total += new_samp;
		avg = moving_avg_buffer->current_total / moving_avg_buffer->head;

		/* Check if the number of averages performed have reached the total */
		if (moving_avg_buffer->head >= moving_avg_buffer->data_buff_length)
		{
			moving_avg_buffer->first_cycle = false;
			moving_avg_buffer->head = 0;
		}
	}
	else
	{
		/* Remove the oldest sample from the average total */
		moving_avg_buffer->current_total -= moving_avg_buffer->data_buff[moving_avg_buffer->tail];
		moving_avg_buffer->tail = (moving_avg_buffer->tail + 1) % moving_avg_buffer->data_buff_length;
		/* Add the new sample to the buffer and the total */
		moving_avg_buffer->data_buff[moving_avg_buffer->head] = new_samp;
		moving_avg_buffer->current_total += new_samp;
		moving_avg_buffer->head = (moving_avg_buffer->head + 1) % moving_avg_buffer->data_buff_length;

		/* Add the new sample to the current average */
		avg = moving_avg_buffer->current_total / moving_avg_buffer->data_buff_length;
	}

	/* Return the average */
	return avg;
}