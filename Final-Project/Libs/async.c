#include "async.h"
#include "ptrBuffer.h"

static struct {
	FLAG(initialized);
} flags = {0};
PtrBuff funcs;

void initAsync(void) {
	if (flags.initialized) return;
	NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(7, 7, 7));
	initPtrBuff(&funcs, 16);
	flags.initialized = 1;
}

void runAsync(asyncFuncPtr callback) {
	addToPtrBuff(&funcs, callback);
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void PendSV_Handler(void) {
	asyncFuncPtr asyncFunc = NULL;
	if (getFromPtrBuff(&funcs, (void *) &asyncFunc) == ERR_EMPTY) {
		SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
	}
	else {
		SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
		(*asyncFunc)();
	}
}
