/**
 * Provides linked list based and array based buffers. All functions work the
 * same on both buffer types, simply pick which you want to use at the buffer's
 * declaration.
 */

#ifndef BUFFER
#define BUFFER

#include <stdlib.h>
#include <stdint.h>
#include "error.h"

#ifndef BUFF_TYPE // Define before including buffer.h to use a custom type
#define BUFF_TYPE uint32_t   // Defaults to uint32_t
#endif

/**
 * Array based implementation, default
 */
typedef struct {
	uint32_t head;            // Index where next item will be placed
	uint32_t tail;            // Index of first item that will be removed
	uint32_t numItems; // Current number of items in buffer
	uint32_t size;              // Capacity of buffer
	BUFF_TYPE * mem;            // Points to beginning of storage array
} Buff;

/**
 * Linked list based implementation
 */
typedef struct {
	Node * next;
	BUFF_TYPE data;
} Node;

typedef struct {
	Node * head;
	Node * tail;
	uint32_t numItems;
	uint32_t size;
} LLBuff;


/**
 * ALLOCATES MEMORY!!! FREE WITH freeBuff
 *
 * @desc    Takes a created buffer and initializes it to the requested length
 * @param   buff - The address of any buffer
 * @param   size - the number of elements to allocate
 * @returns Success or the error that occurred
 */
error initBuff(Buff * buff, uint32_t size);
error initLLBuff(LLBuff * buff, uint32_t size);

/**
 * @desc    Empties the buffer, keeps array, frees nodes in linked list
 * @param   buff - The address of an initialized buffer
 * @returns Success or the error that occurred
 */
error emptyBuff(Buff * buff);
error emptyLLBuff(LLBuff * buff);
/**
 * @desc    Frees any memory allocated from initBuff
 * @param   buff - The address of an initialized buffer
 * @returns Success or the error that occurred
 */
error freeBuff(Buff * buff);
error freeLLBuff(LLBuff * buff);

/**
 * @param   buff - The address of an initialized buffer
 * @returns 1 if the buffer is full, else 0
 */
inline int8_t buffFull(Buff * buff);
inline int8_t llBuffFull(LLBuff * buff);

/**
 * @param   buff - The address of an initialized buffer
 * @returns 1 if the buffer is empty, else 0
 */
inline int8_t buffEmpty(Buff * buff);
inline int8_t llBuffEmpty(LLBuff * buff);

/**
 * @desc    Adds a given item to the buffer if it is not full
 * @param   buff - The address of an initialized buffer
 * @param   item - The item to add
 * @returns Success or the error that occurred
 */
error addToBuff(Buff * buff, BUFF_TYPE item);
error addToLLBuff(LLBuff * buff, BUFF_TYPE item);

/**
 * @desc    Gets the next item from the buffer if it is not empty
 * @param   buff - The address of an initialized buffer
 * @param   container - The address of the variable to put the item in
 * @returns Success or the error that occurred
 */
error getFromBuff(Buff * buff, BUFF_TYPE * container);
error getFromLLBuff(LLBuff * buff, BUFF_TYPE * container);

#endif
