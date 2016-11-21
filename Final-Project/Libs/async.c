#include "async.h"

asyncFuncPtr asyncFunc = NULL;

void initAsync(void) {
	SCB->SHP[10] = 1;
}

void runAsync(asyncFuncPtr callback) {
	asyncFunc = callback;
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void PendSV_Handler(void) {
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
	(*asyncFunc)();
	asyncFunc = NULL;
}
