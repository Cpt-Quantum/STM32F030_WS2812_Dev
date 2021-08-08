#ifndef ADC_H
#define ADC_H

#include <stdint.h>

typedef enum
{
	ADC_BUFFER_HALF_FULL = 0,
	ADC_BUFFER_FULL = 1,
	ADC_TRANSFER_ERROR = 2
} ADC_STATUS_E;

typedef struct
{
	uint32_t channel_select;
	uint16_t *data;
	uint32_t data_length;
	uint16_t clock_div;
	void (*callback)(ADC_STATUS_E);
} ADC_t;

void adc_init(ADC_t *adc_settings);

#endif //ADC_H
