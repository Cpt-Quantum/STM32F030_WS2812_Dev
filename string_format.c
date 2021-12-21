#include <stdint.h>

#include "string_format.h"

void uint32_to_string(char *string, uint32_t val)
{
	/* Add 0x to the first two elements of the string */
	string[0] = '0';
	string[1] = 'x';

	/* Starting with the top 8 bits, and then moving down, append to the string */
	/* the associated ascii represenations of the hex values in val. */
	uint8_to_ascii(&string[2], (val & 0xFF000000) >> 24);
	uint8_to_ascii(&string[4], (val & 0x00FF0000) >> 16);
	uint8_to_ascii(&string[6], (val & 0x0000FF00) >> 8);
	uint8_to_ascii(&string[8], (val & 0x000000FF) >> 0);
}
