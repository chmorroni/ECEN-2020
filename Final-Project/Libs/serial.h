#include <stdint.h>
#include "msp.h"
#include "helpers.h"

#ifndef SERIAL
#define SERIAL

typedef void (*interruptFuncPtr)(void);

/**
 * @desc The function that is called by the interrupt. A function
 *       pointer variable cannot be used as the interrupt needs a
 *       const pointer. PUT THIS IN THE ARRAY IN THE STARTUP FILE!
 */
void eUSCIA0Handler(void);

void enableUART(uint32_t freq, uint32_t baud);
void enableInterruptsUART(interruptFuncPtr callback, uint16_t interrupts);
void inline sendByteUART(uint8_t tx_data);
void sendBytesUART(uint8_t * array, uint32_t length);
void sendStringUART(char * string);
void sendNewlineUART(void);

#endif
