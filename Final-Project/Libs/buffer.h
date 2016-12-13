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
 * Returns true if initialized buffer pointed to by buff is full.
 */
inline int8_t buffFull(Buff * buff);

/**
 * Returns true if initialized buffer pointed to by buff is empty.
 */
inline int8_t buffEmpty(Buff * buff);

/**
 * Returns true if initialized buffer pointed to by buff is initialized.
 */
inline int8_t buffInitialized(Buff * buff);

/**
 * ALLOCATES MEMORY!!! FREE WITH freeBuff
 * Allocates the memory for the uninitialized buffer pointed to by buff to the
 * size indicated by size.
 */
error initBuff(Buff * buff, uint32_t size);

/**
 * Empties the initialized buffer, does not free memory.
 */
error emptyBuff(Buff * buff);

/**
 * Empties the initialized buffer and frees the memory.
 */
error freeBuff(Buff * buff);

/**
 * Adds a given item to the initialized buffer if it is not full, returns the
 * error if it is full.
 */
error addToBuff(Buff * buff, BUFF_TYPE item);

/**
 * Gets the next item from the initialized buffer if it is not empty, returns
 * the error if it is empty. Puts the item in where container points.
 */
error getFromBuff(Buff * buff, BUFF_TYPE * container);

#endif
