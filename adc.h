#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(uint32_t channel_select, uint16_t *data, uint32_t data_length,
				uint16_t clock_div);

#endif	//ADC_H
