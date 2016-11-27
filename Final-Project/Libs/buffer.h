/**
 * Provides a fifo buffer of the type BUFF_TYPE, which can be defined before
 * including the library. If not defined, it defaults to uint32_t.
 */

#ifndef BUFFER
#define BUFFER

#include <stdlib.h>
#include <stdint.h>
#include "error.h"

#ifndef BUFF_TYPE // Define before including buffer.h to use a custom type
#define BUFF_TYPE uint8_t   // Defaults to uint32_t
#endif

/**
 * The buffer struct, create one of these and pass its address to the initBuff
 * function to allocate the space for storage.
 */
typedef struct {
	uint32_t head;     // Index where next item will be placed
	uint32_t tail;     // Index of first item that will be removed
	uint32_t numItems; // Current number of items in buffer
	uint32_t size;     // Capacity of buffer
	BUFF_TYPE * mem;   // Points to beginning of storage array
} Buff;

/**
 * ALLOCATES MEMORY!!! FREE WITH freeBuff
 *
 * @desc  Takes a created buffer and initializes it to the requested length.
 * @param buff - The address of an uninitialized buffer (use freeBuff before
 *               reinitializing an initialized buffer.
 * @param size - The number of elements to store.
 */
error initBuff(Buff * buff, uint32_t size);

/**
 * @desc  Empties the buffer, does not free memory or change size.
 * @param buff - The address of an initialized buffer.
 */
error emptyBuff(Buff * buff);

/**
 * @desc  Frees any memory allocated from initBuff. This deletes all data and
 *        resets the size to 0.
 * @param buff - The address of an initialized buffer.
 */
error freeBuff(Buff * buff);

/**
 * @desc  Returns true if the buffer is full, else false. No error checking is
 *        done.
 * @param buff - The address of an initialized buffer.
 */
inline int8_t buffFull(Buff * buff);

/**
 * @desc  Returns true if the buffer is empty, else false. No error checking is
 *        done.
 * @param buff - The address of an initialized buffer.
 */
inline int8_t buffEmpty(Buff * buff);

/**
 * @desc  Returns true if the buffer is initialized, else false. No error
 *        checking is done.
 * @param buff - The address of an initialized buffer.
 */
inline int8_t buffInitialized(Buff * buff);

/**
 * @desc  Adds a given item to the buffer if it is not full, returns the error
 *        if it is full.
 * @param buff - The address of an initialized buffer.
 * @param item - The item to add.
 */
error addToBuff(Buff * buff, BUFF_TYPE item);

/**
 * @desc  Gets the next item from the buffer if it is not empty, returns the
 *        error if it is empty.
 * @param buff - The address of an initialized buffer.
 * @param container - The address of the variable to put the item in.
 */
error getFromBuff(Buff * buff, BUFF_TYPE * container);

#endif
