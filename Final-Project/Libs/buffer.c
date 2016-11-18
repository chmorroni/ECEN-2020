#include <stdlib.h>
#include "buffer.h"

/**
 * @desc  Returns true if the buffer is full, else false. No error checking is
 *        done.
 * @param buff - The address of an initialized buffer.
 */
inline int8_t buffFull(Buff * buff) {
	return buff->numItems >= buff->size;
}

/**
 * @desc  Returns true if the buffer is empty, else false. No error checking is
 *        done.
 * @param buff - The address of an initialized buffer.
 */
inline int8_t buffEmpty(Buff * buff) {
	return !buff->numItems;
}

/**
 * @desc  Returns true if the buffer is initialized, else false. No error
 *        checking is done.
 * @param buff - The address of an initialized buffer.
 */
inline int8_t buffInitialized(Buff * buff) {
	return !!buff->size;
}

/**
 * ALLOCATES MEMORY!!! FREE WITH freeBuff
 *
 * @desc  Takes a created buffer and initializes it to the requested length.
 * @param buff - The address of an uninitialized buffer (use freeBuff before
 *               reinitializing an initialized buffer.
 * @param size - The number of elements to store.
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
 * @desc  Empties the buffer, does not free memory or change size.
 * @param buff - The address of an initialized buffer.
 */
error emptyBuff(Buff * buff) {
	if (!buff) return ERR_NULL_PTR;
	if (!buffInitialized(buff)) return ERR_UNINITIALIZED;
	buff->head = buff->tail = buff->numItems = 0; // Reset everything
	return ERR_NO;
}

/**
 * @desc  Frees any memory allocated from initBuff. This deletes all data and
 *        resets the size to 0.
 * @param buff - The address of an initialized buffer.
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
 * @desc  Adds a given item to the buffer if it is not full, returns the error
 *        if it is full.
 * @param buff - The address of an initialized buffer.
 * @param item - The item to add.
 */
error addToBuff(Buff * buff, BUFF_TYPE item) {
	if (!buff) return ERR_NULL_PTR;
	if (!buffInitialized(buff)) return ERR_UNINITIALIZED;
	if (buffFull(buff)) return ERR_FULL;
	buff->mem[buff->tail] = item;
	buff->numItems++;
	buff->tail = (buff->tail + 1) % buff->size;
	return ERR_NO;
}

/**
 * @desc  Gets the next item from the buffer if it is not empty, returns the
 *        error if it is empty.
 * @param buff - The address of an initialized buffer.
 * @param container - The address of the variable to put the item in.
 */
error getFromBuff(Buff * buff, BUFF_TYPE * container) {
	if (!buff || !container) return ERR_NULL_PTR;
	if (!buffInitialized(buff)) return ERR_UNINITIALIZED;
	if (buffEmpty(buff)) return ERR_EMPTY;
	*container = buff->mem[buff->head];
	buff->numItems--;
	buff->head = (buff->head + 1) % buff->size;
	return ERR_NO;
}
