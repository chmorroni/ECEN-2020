#include <stdlib.h>
#include "buffer.h"

/**
 * @param   buff - The address of an initialized buffer
 * @returns 1 if the buffer is full, else 0
 */
inline int8_t buffFull(Buff * buff) {
	return buff->numItems >= buff->size;
}
inline int8_t llBuffFull(LLBuff * buff) {
	return buff->numItems >= buff->size;
}

/**
 * @param   buff - The address of an initialized buffer
 * @returns 1 if the buffer is empty, else 0
 */
inline int8_t buffEmpty(Buff * buff) {
	return !buff->numItems;
}
inline int8_t llBuffEmpty(LLBuff * buff) {
	return !buff->numItems;
}

/**
 * TODO
 */
inline static int8_t buffInitialized(Buff * buff) {
	return !buff->size;
}
static inline int8_t llBuffInitialized(LLBuff * buff) {
	return !buff->size;
}

/**
 * ARRAY BASED ALLOCATES MEMORY!!! FREE WITH freeBuff
 *
 * @desc    Takes a created buffer and initializes it to the requested length
 * @param   buff - The address of any buffer
 * @param   size - the number of elements to allocate
 * @returns Success or the error that occurred
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
error initLLBuff(LLBuff * buff, uint32_t size) {
	if (!buff) return ERR_NULL_PTR;
	buff->head = NULL;
	buff->tail = NULL;
	buff->numItems = 0;
	buff->size = size;
	return ERR_NO;
}

/**
 * @desc    Empties the buffer, does not delete the buffer
 * @param   buff - The address of an initialized buffer
 * @returns Success or the error that occurred
 */
error emptyBuff(Buff * buff) {
	if (!buff) return ERR_NULL_PTR;
	if (!buffInitialized(buff)) return ERR_UNINITIALIZED;
	buff->head = buff->tail = buff->numItems = 0; // Reset everything
	return ERR_NO;
}
error emptyLLBuff(LLBuff * buff) {
	Node * current, * next;
	if (!buff) return ERR_NULL_PTR;
	if (!buff->size) return ERR_UNINITIALIZED;
	current = buff->head;
	for (current = buff->head; current; current = next) {
		next = current->next;
		free(current);
	}
	buff->head = NULL;
	buff->tail = NULL;
	buff->numItems = 0;
	return ERR_NO;
}

/**
 * @desc    Frees any memory allocated from initBuff
 * @param   buff - The address of an initialized buffer
 * @returns Success or the error that occurred
 */
error freeBuff(Buff * buff) {
	if (!buff) return ERR_NULL_PTR;
	if (!buffInitialized(buff)) return ERR_UNINITIALIZED;
	free(buff->mem);
	buff->mem = NULL;
	buff->head = buff->tail = buff->numItems = buff->size = 0; // Reset everything
		return ERR_NO;
}
error freeLLBuff(LLBuff * buff) {
	if (!buff) return ERR_NULL_PTR;
	if (!llBuffInitialized(buff)) return ERR_UNINITIALIZED;
	emptyLLBuff(buff);
	buff->size = 0;
	return ERR_NO;
}

/**
 * @desc    Adds a given item to the buffer if it is not full
 * @param   buff - The address of an initialized buffer
 * @param   item - The item to add
 * @returns Success or the error that occurred
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
error addToLLBuff(LLBuff * buff, BUFF_TYPE item) {
	Node * newNode = NULL;
	if (!buff) return ERR_NULL_PTR;
	if (!llBuffInitialized(buff)) return ERR_UNINITIALIZED;
	if (llBuffFull(buff)) return ERR_FULL;
	if (!(newNode = malloc(sizeof (Node)))) return ERR_OUT_OF_MEM;
	newNode->next = NULL;
	newNode->data = item;
	if (llBuffEmpty(buff)) buff->head = newNode;
	else buff->tail->next = newNode;
	buff->tail = newNode;
	buff->numItems++;
	return ERR_NO;
}

/**
 * @desc    Gets the next item from the buffer if it is not empty
 * @param   buff - The address of an initialized buffer
 * @param   container - The address of the variable to put the item in
 * @returns Success or the error that occurred
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
error getFromLLBuff(LLBuff * buff, BUFF_TYPE * container) {
	Node * newHead = NULL;
	if (!buff || !container) return ERR_NULL_PTR;
	if (!llBuffInitialized(buff)) return ERR_UNINITIALIZED;
	if (llBuffEmpty(buff)) return ERR_EMPTY;
	*container = buff->head->data;
	newHead = buff->head->next;
	free(buff->head);
	buff->head = newHead;
	if (buff->numItems == 1) buff->tail = NULL;
	buff->numItems--;
	return ERR_NO;
}
