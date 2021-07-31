#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	volatile char *buffer;
	uint16_t length;
	uint16_t head;
	uint16_t tail;
	bool buffer_empty;
	bool buffer_full;
} ringbuffer_t;

/* TODO: Check whether using conditional logic is faster than using modulo arithmetic */
/* to update the head. */
static inline void ringbuffer_write(ringbuffer_t *buf, char new_entry)
{
	buf->buffer[buf->head] = new_entry;
	buf->head = (buf->head + 1) % buf->length;
	buf->buffer_empty = false;
	/* If the head has now reached the tail, then set the buffer full flag */
	if (buf->head == buf->tail)
	{
		buf->buffer_full = true;
	}
}
static inline char ringbuffer_read(ringbuffer_t *buf)
{
	/* Set empty buffer flag if head = tail and the buffer isn't full */
	if (buf->head == buf->tail && !(buf->buffer_full))
	{
		buf->buffer_empty = true;
		return '\0';
	}
	char read = buf->buffer[buf->tail];
	buf->tail = (buf->tail + 1) % buf->length;
	buf->buffer_empty = false;
	buf->buffer_full = false;
	return read;
}

static inline void ringbuffer_write_string(ringbuffer_t *buf, char *string, uint32_t length)
{
	for(uint32_t i = 0; i < length; i++)
	{
		ringbuffer_write(buf, string[i]);
	}
}

static inline void ringbuffer_read_string(ringbuffer_t *buf, char *string, uint32_t length)
{
	for(uint32_t i = 0; i < length; i++)
	{
		string[i] = ringbuffer_read(buf);
	}
}

static inline uint16_t ringbuffer_get_length(ringbuffer_t buf)
{
	if (buf.head > buf.tail)
	{
		return (buf.head - buf.tail + 1);
	}
	else if (buf.head < buf.tail)
	{
		return (buf.head + (buf.length - buf.tail) + 1);
	}
	else
	{
		return 0;
	}
}

#endif //CIRCULAR_BUFFER_H
