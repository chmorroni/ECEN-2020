/**
 * Provides a fifo buffer of the type BUFF_TYPE, which can be defined before
 * including the library. If not defined, it defaults to uint32_t.
 */

#ifndef PTR_BUFFER
#define PTR_BUFFER

#include <stdlib.h>
#include <stdint.h>
#include "error.h"

#ifndef PTR_BUFF_TYPE // Define before including buffer.h to use a custom type
#define PTR_BUFF_TYPE void *   // Defaults to uint32_t
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
	PTR_BUFF_TYPE * mem;   // Points to beginning of storage array
} PtrBuff;

/**
 * Returns true if initialized buffer pointed to by buff is full.
 */
inline int8_t ptrBuffFull(PtrBuff * buff);

/**
 * Returns true if initialized buffer pointed to by buff is empty.
 */
inline int8_t ptrBuffEmpty(PtrBuff * buff);

/**
 * Returns true if initialized buffer pointed to by buff is initialized.
 */
inline int8_t ptrBuffInitialized(PtrBuff * buff);

/**
 * ALLOCATES MEMORY!!! FREE WITH freeBuff
 * Allocates the memory for the uninitialized buffer pointed to by buff to the
 * size indicated by size.
 */
error initPtrBuff(PtrBuff * buff, uint32_t size);

/**
 * Empties the initialized buffer, does not free memory.
 */
error emptyPtrBuff(PtrBuff * buff);

/**
 * Empties the initialized buffer and frees the memory.
 */
error freePtrBuff(PtrBuff * buff);

/**
 * Adds a given item to the initialized buffer if it is not full, returns the
 * error if it is full.
 */
error addToPtrBuff(PtrBuff * buff, PTR_BUFF_TYPE item);

/**
 * Gets the next item from the initialized buffer if it is not empty, returns
 * the error if it is empty. Puts the item in where container points.
 */
error getFromPtrBuff(PtrBuff * buff, PTR_BUFF_TYPE * container);

#endif
