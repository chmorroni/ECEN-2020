#include <stdint.h>
#include "msp.h"
#include "helpers.h"

#ifndef UART
#define UART

typedef void (*interruptFuncPtr)(void);

void enableUART(uint32_t freq, uint32_t baud);
void enableUARTInterrupts(interruptFuncPtr callback, uint16_t interrupts);
void inline sendByteUART(uint8_t tx_data);
void sendBytesUART(uint8_t * array, uint32_t length);
void sendStringUART(char * string);
void sendNewlineUART(void);

#endif
