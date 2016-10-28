#include <stdlib.h>
#include <stdint.h>
#ifndef BUFFER_FIFO
#define BUFFER_FIFO

typedef enum { // BE == Buffer Error
	BE_EMPTY = -4,
	BE_FULL = -3,
	BE_NULL_PTR = -2,
	BE_LOW_MEM = -1,
	BE_NO_ERR = 0
} bufError_T;

typedef struct CircBuf{
	uint32_t *head;
	uint32_t *tail;
	volatile uint32_t num_items;
	uint32_t length;
	uint32_t *buffer;
} CircBuf_T;

bufError_T initializeBuffer(CircBuf_T *buf, uint32_t length);
void clearBuffer(CircBuf_T *buf);
void deleteBuffer(CircBuf_T *buf);
int8_t bufferFull(CircBuf_T *buf);
int8_t bufferEmpty(CircBuf_T *buf);
bufError_T addItem(CircBuf_T *buf, uint32_t item);
bufError_T removeItem(CircBuf_T *buf, uint32_t *val);

#endif
