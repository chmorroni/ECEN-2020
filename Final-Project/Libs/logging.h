#ifndef MSP_LOG
#define MSP_LOG

#include <stdint.h>
#include "msp.h"
#include "error.h"

typedef void (* commandFuncPtr)(uint8_t received);
typedef void (* getLineFuncPtr)(char * line, error err);
typedef void (* getCharFuncPtr)(char received);
typedef void (* getOptionFuncPtr)(uint32_t option);
typedef void (* getNumberFuncPtr)(int64_t number);

void eUSCIUARTHandler(void);

/**
 * Configures logging on the given module at the requested baud rate. Once pins
 * are mapped, use the start logging function to allow logging.
 */
error configLogging(EUSCI_A_Type * module, uint32_t baud);
error startLogging(IRQn_Type uartIRQ);
/**
 * Enables one character commands, takes a callback to run when a command is
 * received
 */
error enableCommands(commandFuncPtr callback);
/**
 * Calls callback asynchronously when a character is received
 */
error getChar(getCharFuncPtr callback);
/**
 * Calls callback asynchronously when a line is received
 */
error getLine(getLineFuncPtr callback);
error getNumberInRange(getNumberFuncPtr callback, int64_t min, int64_t max);
/**
 * Calls callback asynchronously when a line is received
 */
error getOption(getOptionFuncPtr callback, uint32_t optionsLen, char ** optionsArr);
error printChar(char letter);
error printNewline(void);
error printString(char * string);
error printArray(uint32_t len, char ** strArr);
error printWrap(char * string, uint8_t lvl1Indent, uint8_t lvlNIndent, uint8_t lvl1CharsPrinted);
error log(char * string);
error readyForCommands(void);
error pauseCommands(void);

#endif
