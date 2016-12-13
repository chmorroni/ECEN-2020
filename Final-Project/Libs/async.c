#include "async.h"
#include "ptrBuffer.h"

static struct {
	FLAG(initialized);
} flags = {0};
static PtrBuff funcs;

/**
 * Initialize necessary buffer and priority to ensure PendSV does not block
 * other interrupts.
 */
void initAsync(void) {
	if (flags.initialized) return;
	// Set the PendSV to the lowest possible priority
	NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(7, 7, 7));
	initPtrBuff(&funcs, 16);
	flags.initialized = 1;
}

/**
 * Add the function to the buffer of funcs to run and start running.
 */
void runAsync(asyncFuncPtr callback) {
	addToPtrBuff(&funcs, callback);
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/**
 * Interrupt handler
 */
void PendSV_Handler(void) {
	asyncFuncPtr asyncFunc = NULL;
	// No more functions need to be run asynchronously
	if (getFromPtrBuff(&funcs, (void *) &asyncFunc) == ERR_EMPTY) {
		SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
	}
	// There are still more things to run
	else {
		SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
		(*asyncFunc)();
	}
}
