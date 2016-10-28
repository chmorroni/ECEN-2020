#include "buffer_fifo.h"

bufError_T initializeBuffer(CircBuf_T *buf, uint32_t length){
	if (!buf) return BE_NULL_PTR;
	uint32_t *buffer = (uint32_t *) malloc(length * sizeof (uint32_t));
	if (!buffer) return BE_LOW_MEM;
	buf->head = buffer;
	buf->tail = buffer;
	buf->num_items = 0;
	buf->length= length;
	buf->buffer = buffer;
	return BE_NO_ERR;
}

void clearBuffer(CircBuf_T *buf){
	buf->head = buf->buffer;
	buf->tail = buf->buffer;
	buf->num_items = 0;
}

void deleteBuffer(CircBuf_T *buf){
	free(buf->buffer);
}

int8_t bufferFull(CircBuf_T *buf){
	return buf->num_items >= buf->length;
}

int8_t bufferEmpty(CircBuf_T *buf){
	return !buf->num_items;
}

bufError_T addItem(CircBuf_T *buf, uint32_t item){
	if (!buf) return BE_NULL_PTR;
	if (bufferFull(buf)) {
		buf->tail = buf->buffer + ((buf->tail - buf->buffer) + 1) % buf->length;
	}
	else buf->num_items++;
	*(buf->head) = item;
	buf->head = buf->buffer + ((buf->head - buf->buffer) + 1) % buf->length;
	return bufferFull(buf) ? BE_FULL : BE_NO_ERR;
}

bufError_T removeItem(CircBuf_T *buf, uint32_t *val){
	if (!buf || !val) return BE_NULL_PTR;
	if (bufferEmpty(buf)) return BE_EMPTY;
	*val = *(buf->tail);
	buf->tail = buf->buffer + ((buf->tail - buf->buffer) + 1) % buf->length;
	buf->num_items--;
	return BE_NO_ERR;
}
