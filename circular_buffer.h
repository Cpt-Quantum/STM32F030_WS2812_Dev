#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>

typedef struct {
	volatile char *buffer;
	uint16_t length;
	uint16_t head;
	uint16_t tail;
} ringbuffer_t;

/* TODO: Check whether using conditional logic is faster than using modulo arithmetic */
/* to update the head. */
static inline void ringbuffer_write(ringbuffer_t *buf, char new_entry)
{
	buf->buffer[buf->head] = new_entry;
	buf->head = (buf->head + 1) % buf->length;
}
static inline char ringbuffer_read(ringbuffer_t *buf)
{
	/* Return null for empty buffer */
	if (buf->head == buf->tail)
	{
		return '\0';
	}
	char read = buf->buffer[buf->tail];
	buf->tail = (buf->tail + 1) % buf->length;
	return read;
}

static inline void ringbuffer_write_string(ringbuffer_t *buf, char *string)
{
	uint16_t i = 0;
	while(1)
	{
		/* Check for null as this indicates the buffer is empty */
		if (string[i] == '\0')
		{
			break;
		}
		else
		{
			ringbuffer_write(buf, string[i]);
		}
		i++;
	}
}

static inline void ringbuffer_read_string(ringbuffer_t *buf, char *string)
{
	uint16_t i = 0;
	while(1)
	{
		string[i] = ringbuffer_read(buf);
		/* Check for end of buffer and exit */
		if (string[i] == '\0')
		{
			break;
		}
		i++;
	}
}

#endif //CIRCULAR_BUFFER_H
