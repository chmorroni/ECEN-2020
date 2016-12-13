#include <stdlib.h>
#include "buffer.h"
#include "msp.h"

/**
 * Returns true if initialized buffer pointed to by buff is full.
 */
inline int8_t buffFull(Buff * buff) {
	return buff->numItems >= buff->size;
}

/**
 * Returns true if initialized buffer pointed to by buff is empty.
 */
inline int8_t buffEmpty(Buff * buff) {
	return !buff->numItems;
}

/**
 * Returns true if initialized buffer pointed to by buff is initialized.
 */
inline int8_t buffInitialized(Buff * buff) {
	return !!buff->size;
}

/**
 * ALLOCATES MEMORY!!! FREE WITH freeBuff
 * Allocates the memory for the uninitialized buffer pointed to by buff to the
 * size indicated by size.
 */
error initBuff(Buff * buff, uint32_t size) {
	if (!buff) return ERR_NULL_PTR;
	BUFF_TYPE * mem = (BUFF_TYPE *) malloc(size * sizeof (BUFF_TYPE));
	if (!mem) return ERR_OUT_OF_MEM;
	buff->head = 0;
	buff->tail = 0;
	buff->numItems = 0;
	buff->size = size;
	buff->mem = mem;
	return ERR_NO;
}

/**
 * Empties the initialized buffer, does not free memory.
 */
error emptyBuff(Buff * buff) {
	if (!buff) return ERR_NULL_PTR;
	if (!buffInitialized(buff)) return ERR_UNINITIALIZED;
	buff->head = buff->tail = buff->numItems = 0; // Reset everything
	return ERR_NO;
}

/**
 * Empties the initialized buffer and frees the memory.
 */
error freeBuff(Buff * buff) {
	if (!buff) return ERR_NULL_PTR;
	if (!buffInitialized(buff)) return ERR_UNINITIALIZED;
	free(buff->mem);
	buff->mem = NULL;
	buff->head = buff->tail = buff->numItems = buff->size = 0; // Reset everything
		return ERR_NO;
}

/**
 * Adds a given item to the initialized buffer if it is not full, returns the
 * error if it is full.
 */
error addToBuff(Buff * buff, BUFF_TYPE item) {
	if (!buff) return ERR_NULL_PTR;
	if (!buffInitialized(buff)) return ERR_UNINITIALIZED;
	if (buffFull(buff)) return ERR_FULL;
	__disable_interrupts();
	buff->mem[buff->tail] = item;
	buff->numItems++;
	buff->tail = (buff->tail + 1) % buff->size;
	__enable_interrupts();
	return ERR_NO;
}

/**
 * Gets the next item from the initialized buffer if it is not empty, returns
 * the error if it is empty. Puts the item in where container points.
 */
error getFromBuff(Buff * buff, BUFF_TYPE * container) {
	if (!buff || !container) return ERR_NULL_PTR;
	if (!buffInitialized(buff)) return ERR_UNINITIALIZED;
	if (buffEmpty(buff)) return ERR_EMPTY;
	__disable_interrupts();
	*container = buff->mem[buff->head];
	buff->numItems--;
	buff->head = (buff->head + 1) % buff->size;
	__enable_interrupts();
	return ERR_NO;
}
