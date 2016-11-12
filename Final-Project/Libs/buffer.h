#include <stdlib.h>
#include <stdint.h>
#ifndef BUFFER
#define BUFFER

// Define BUFF_TYPE before including buffer.h to use a custom type
#ifndef BUFF_TYPE
// Defaults to uint32_t
#define BUFF_TYPE uint32_t
#endif

typedef enum {
	BUFF_UNINITIALIZED = -5,   // Tried to use an uninitialized buffer
	BUFF_EMPTY = -4,           // Tried to get an item, but the buffer is empty
	BUFF_FULL = -3,            // Tried to add an item, but the buffer is full
	BUFF_NULL_PTR = -2,        // Passed a null pointer to a required parameter
	BUFF_LOW_MEM = -1,         // Requested larger buffer than can be allocated
	BUFF_NO_ERR = 0            // All went well
} buffErr;

typedef struct {
	uint32_t head;              // index where next item will be placed
	uint32_t tail;              // index of first item that will be removed
	volatile uint32_t numItems; // Current number of items in buffer
	uint32_t size;              // Capacity of buffer
	BUFF_TYPE * mem;            // Points to beginning of storage array
} Buff;

/**
 * ALLOCATES MEMORY!!! FREE WITH freeBuff
 *
 * @desc    Takes a created buffer and initializes it to the requested length
 * @param   buff - The address of any buffer
 * @param   size - the number of elements to allocate
 * @returns Success or the error that occurred
 */
buffErr initBuff(Buff * buff, uint32_t size);

/**
 * @desc    Empties the buffer, does not delete the buffer
 * @param   buff - The address of an initialized buffer
 * @returns Success or the error that occurred
 */
buffErr emptyBuff(Buff * buff);

/**
 * @desc    Frees any memory allocated from initBuff
 * @param   buff - The address of an initialized buffer
 * @returns Success or the error that occurred
 */
buffErr freeBuff(Buff * buff);

/**
 * @param   buff - The address of an initialized buffer
 * @returns 1 if the buffer is full, else 0
 */
inline int8_t buffFull(Buff * buff);

/**
 * @param   buff - The address of an initialized buffer
 * @returns 1 if the buffer is empty, else 0
 */
inline int8_t buffEmpty(Buff * buff);

/**
 * @desc    Adds a given item to the buffer if it is not full
 * @param   buff - The address of an initialized buffer
 * @param   item - The item to add
 * @returns Success or the error that occurred
 */
buffErr addToBuff(Buff * buff, BUFF_TYPE item);

/**
 * @desc    Gets the next item from the buffer if it is not empty
 * @param   buff - The address of an initialized buffer
 * @param   container - The address of the variable to put the item in
 * @returns Success or the error that occurred
 */
buffErr getFromBuff(Buff * buff, BUFF_TYPE * container);

#endif
