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

/**
 * Handler to receive chars. Place this function in the startup file in the
 * place of the EUSCIA handler.
 */
void eUSCIUARTHandler(void);

/**
 * Configures logging on the given module at the requested baud rate. Once pins
 * are mapped, use the start logging function to allow logging.
 */
error configLogging(EUSCI_A_Type * module, uint32_t baud);

/**
 * Actually enables the proper interrupts once port mapping is complete.
 */
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
 * Calls callback asynchronously when a line is received (ending in '\r'.
 */
error getLine(getLineFuncPtr callback);

/**
 * Takes an array of options. Prints them out and allows the user to select one
 * by going through the list and clicking enter on the selected one. Calls
 * callback asynchronously on the option.
 */
error getOption(getOptionFuncPtr callback, uint32_t optionsLen, char ** optionsArr);

/**
 * Prompts for a number to be entered at the console.
 */
error getNumberInRange(getNumberFuncPtr callback, int64_t min, int64_t max);

/**
 * Prints a char. Blocking if getting.
 */
error printChar(char letter);

/**
 * Prints '\n', '\r'.
 */
error printNewline(void);

/**
 * Iterates through string, printing characters.
 */
error printString(char * string);

/**
 * Prints array of characters, based on a length.
 */
error printArray(uint32_t len, char ** strArr);

/**
 * Prints a string and wraps at 80 chars.
 * lvl1Indent - Spaces to put before the first line
 * lvlNIndent - Spaces to put before every line except the first
 * lvl1CharsPrinted - The chars printed since the last newline
 */
error printWrap(char * string, uint8_t lvl1Indent, uint8_t lvlNIndent, uint8_t lvl1CharsPrinted);

/**
 * Prints a time stamp and wraps the string at 80 chars. Allows nested logging
 * by separating them with break chars.
 */
error log(char * string);

/**
 * Run to enable the '$ ' line at the bottom of the terminal and allow the
 * callback to be run.
 */
error readyForCommands(void);

/**
 * Run when the terminal is busy printing to hide the '$ '.
 */
error pauseCommands(void);

#endif
