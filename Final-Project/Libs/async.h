#ifndef MSP_ASYNC
#define MSP_ASYNC

#include <stdint.h>
#include "msp.h"
#include "helpers.h"

typedef void (* asyncFuncPtr)(void);

/**
 * Initialize necessary buffer and priority to ensure PendSV does not block
 * other interrupts.
 */
void initAsync(void);

/**
 * Add the function to the buffer of funcs to run and start running.
 */
void runAsync(asyncFuncPtr callback);

/**
 * Interrupt handler
 */
void PendSV_Handler(void);

#endif
