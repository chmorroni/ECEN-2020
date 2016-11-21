#ifndef MSP_LOG
#define MSP_LOG

#include <stdint.h>
#include "msp.h"
#include "error.h"

typedef void (* commandFuncPtr)(uint8_t received);
typedef void (* getLinePtr)(char * line, error err);
typedef void (* getCharPtr)(char received);
typedef enum {
	NO_INDENT,
	INDENT
} indentation;
typedef enum {
	SEND_ASYNC,
	SEND_INTERRUPT_SAFE
} sendMode;

void eUSCIUARTHandler(void);

/**
 * @desc Enables UART on the specified module and sets up the buffer.
 */
error configLogging(EUSCI_A_Type * module, uint32_t baud, uint32_t size);

error startLogging(IRQn_Type uartIRQ);

error enableCommands(commandFuncPtr callback);

error getChar(getCharPtr callback);

error getLine(getLinePtr callback);

error printChar(char letter, sendMode mode);

error printNewline(sendMode mode);

error printString(char * string, sendMode mode);

error printWrap(char * string, uint8_t lvl1Indent, uint8_t lvlNIndent, uint8_t lvl1CharsPrinted, sendMode mode);

error log(char * string, sendMode mode);

void enableTimestamps(void);
void disableTimestamps(void);

#endif
