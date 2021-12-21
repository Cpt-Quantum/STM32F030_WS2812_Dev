#ifndef STRING_FORMAT_H
#define STRING_FORMAT_H

#include <stdint.h>

static inline char nibble_to_ascii(uint8_t nibble)
{
	char ret = 0;
	switch (nibble)
	{
	case 0:
		ret = '0';
		break;
	case 1:
		ret = '1';
		break;
	case 2:
		ret = '2';
		break;
	case 3:
		ret = '3';
		break;
	case 4:
		ret = '4';
		break;
	case 5:
		ret = '5';
		break;
	case 6:
		ret = '6';
		break;
	case 7:
		ret = '7';
		break;
	case 8:
		ret = '8';
		break;
	case 9:
		ret = '9';
		break;
	case 10:
		ret = 'A';
		break;
	case 11:
		ret = 'B';
		break;
	case 12:
		ret = 'C';
		break;
	case 13:
		ret = 'D';
		break;
	case 14:
		ret = 'E';
		break;
	case 15:
		ret = 'F';
		break;
	}
	return ret;
}

static inline void uint8_to_ascii(char *string, uint8_t val)
{
	/* Handle the lower 4 bits first */
	string[1] = nibble_to_ascii(val & 0x0F);
	/* Now the top 4 bits */
	string[0] = nibble_to_ascii((val & 0xF0) >> 4);
}

void uint32_to_string(char *string, uint32_t val);

#endif /* STRING_FORMAT_H */
