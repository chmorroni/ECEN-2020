#include "buffer.h"

/**
 * ALLOCATES MEMORY!!! FREE WITH freeBuff
 *
 * @desc    Takes a created buffer and initializes it to the requested length
 * @param   buff - The address of any buffer
 * @param   size - the number of elements to allocate
 * @returns Success or the error that occurred
 */
buffErr initBuff(Buff * buff, uint32_t size) {
	if (!buff) return BUFF_NULL_PTR;
	BUFF_TYPE * mem = (BUFF_TYPE *) malloc(size * sizeof (BUFF_TYPE));
	if (!mem) return BUFF_LOW_MEM;
	buff->head = 0;
	buff->tail = 0;
	buff->numItems = 0;
	buff->size = size;
	buff->mem = mem;
	return BUFF_NO_ERR;
}

/**
 * @desc    Empties the buffer, does not delete the buffer
 * @param   buff - The address of an initialized buffer
 * @returns Success or the error that occurred
 */
buffErr emptyBuff(Buff * buff) {
	if (!buff) return BUFF_NULL_PTR;
	buff->head = buff->numItems = 0; // Reset everything
	return BUFF_NO_ERR;
}

/**
 * @desc    Frees any memory allocated from initBuff
 * @param   buff - The address of an initialized buffer
 * @returns Success or the error that occurred
 */
buffErr freeBuff(Buff * buff) {
	if (!buff) return BUFF_NULL_PTR;
	free(buff->mem);
	return BUFF_NO_ERR;
}

/**
 * @param   buff - The address of an initialized buffer
 * @returns 1 if the buffer is full, else 0
 */
inline int8_t buffFull(Buff * buff) {
	return buff->numItems >= buff->size;
}

/**
 * @param   buff - The address of an initialized buffer
 * @returns 1 if the buffer is empty, else 0
 */
inline int8_t buffEmpty(Buff * buff) {
	return !buff->numItems;
}

/**
 * @desc    Adds a given item to the buffer if it is not full
 * @param   buff - The address of an initialized buffer
 * @param   item - The item to add
 * @returns Success or the error that occurred
 */
buffErr addToBuff(Buff * buff, BUFF_TYPE item) {
	if (!buff) return BUFF_NULL_PTR;
	if (!buff->mem) return BUFF_UNINITIALIZED;
	if (buffFull(buff)) return BUFF_FULL;
	buff->mem[buff->head] = item;
	buff->numItems++;
	buff->head = (buff->head + 1) % buff->size;
	return BUFF_NO_ERR;
}

/**
 * @desc    Gets the next item from the buffer if it is not empty
 * @param   buff - The address of an initialized buffer
 * @param   container - The address of the variable to put the item in
 * @returns Success or the error that occurred
 */
buffErr getFromBuff(Buff * buff, BUFF_TYPE * container) {
	if (!buff || !container) return BUFF_NULL_PTR;
	if (!buff->mem) return BUFF_UNINITIALIZED;
	if (buffEmpty(buff)) return BUFF_EMPTY;
	*container = buff->mem[buff->tail];
	buff->numItems--;
	buff->tail = (buff->tail + 1) % buff->size;
	return BUFF_NO_ERR;
}
