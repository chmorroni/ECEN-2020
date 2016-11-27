#include <stdlib.h>
#include "logging.h"
#include "serial.h"
#include "timers.h"
#include "helpers.h"
#include "async.h"

static EUSCI_A_Type * uartModule = NULL;
static struct {
	commandFuncPtr command;
	getLineFuncPtr getLine;
	getCharFuncPtr getChar;
	getOptionFuncPtr getOption;
	getNumberFuncPtr getNumber;
} callbacks = {0};
static struct {
	FLAG(unhandledCommand); // Command received in the middle of a log
	FLAG(readyForCommand);
} flags;
static enum { // Current get operation
	GETTING_NO,
	GETTING_LINE,
	GETTING_CHAR,
	GETTING_OPTION
} getting;
char ** options = NULL; // For getOption
uint32_t numOptions = 0;
uint32_t currentOption = 0;
int64_t maxNum = 0;
int64_t minNum = 0;
static uint8_t lastRX = 0; // last received char
static char logBreakOpeners[] = {'<', '{', '['}; // Opening chars to iterate through for nested logs.
static char logBreakClosers[] = {'>', '}', ']'}; // Closing ^...

/* Prototypes of internal functions */
/**
 * Prints a character as soon as the buffer is empty. Does not wait for getting
 * operations to complete.
 */
static inline void _printCharNB(char letter);
static inline void _printChar(char letter);
static inline void _printNewlineNB(void);
static inline void _printNewline(void);
static inline void _printString(char * string);

/**
 * Prints a string and wraps at 80 chars.
 */
static inline void _printWrap(char * string, uint8_t lvl1Indent, uint8_t lvlNIndent, uint8_t lvl1CharsPrinted);
static void _stopGettingLine(void);
static void inline _print80NB(char * string, uint8_t charsPrinted);
static void _rxHandler();

/**
 * Configures logging on the given module at the requested baud rate. Once pins
 * are mapped, use the start logging function to allow logging.
 */
error configLogging(EUSCI_A_Type * module, uint32_t baud) {
	if (!module) return ERR_NULL_PTR;
	error err = ERR_NO;
	err = configUART(module, baud);
	if (err != ERR_NO) return err;
	uartModule = module;
	initAsync();
	return err;
}

error startLogging(IRQn_Type uartIRQ) {
	if (!uartModule) return ERR_UNINITIALIZED;
	error err = ERR_NO;
	err = startUART(uartModule);
	uartModule->IE = 0;
	NVIC_EnableIRQ(uartIRQ);
	initRTC();
	return err;
}

/**
 * Enables one character commands, takes a callback to run when a command is
 * received
 */
error enableCommands(commandFuncPtr callback) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	uartModule->IE |= EUSCI_A_IE_RXIE;
	callbacks.command = callback;
	readyForCommands();
	return ERR_NO;
}

/**
 * Calls callback asynchronously when a character is received
 */
error getChar(getCharFuncPtr callback) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	if (getting) return ERR_BUSY;
	callbacks.getChar = callback;
	uartModule->IE |= EUSCI_A_IE_RXIE;
	getting = GETTING_CHAR;
	return ERR_NO;
}

/**
 * Calls callback asynchronously when a line is received
 */
error getLine(getLineFuncPtr callback) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	if (getting) return ERR_BUSY;
	callbacks.getLine = callback;
	uartModule->IE |= EUSCI_A_IE_RXIE;
	getting = GETTING_LINE;
	return ERR_NO;
}

/**
 * Calls callback asynchronously when a line is received
 */
error getOption(getOptionFuncPtr callback, uint32_t optionsLen, char ** optionsArr) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	if (getting) return ERR_BUSY;
	if (optionsLen < 1) return ERR_PARAM_OUT_OF_BOUNDS;
	error err = ERR_NO;
	err = printArray(optionsLen, optionsArr); // Only thing that can go wrong now is printing the array
	if (err) return err;
	_printChar('\r');
	_printChar('>');
	_print80NB(optionsArr[0], 1);
	callbacks.getOption = callback;
	uartModule->IE |= EUSCI_A_IE_RXIE;
	getting = GETTING_OPTION;
	options = optionsArr;
	numOptions = optionsLen;
	currentOption = 0;
	return ERR_NO;
}

void _getLineNumberHandler(char * line, error err) {
	int64_t number = 0;
	if (err) { // Catches line = NULL
		printString("An error occurred getting the number. ERROR: ");
		printWrap(errorToStr(err), 0, 0, 44);
	}
	else {
		err = strToInt64(line, &number);
		if (err) {
			printString("An error occurred parsing the number. ERROR: ");
			printWrap(errorToStr(err), 0, 0, 44);
		}
		else if (number < minNum || number > maxNum) {
			printWrap("Number out of range.", 0, 0, 0);
			err = ERR_PARAM_OUT_OF_BOUNDS;
		}
	}
	if (line) free(line);
	if (!err) (*callbacks.getNumber)(number);
	else getLine(&_getLineNumberHandler);
}

error getNumberInRange(getNumberFuncPtr callback, int64_t min, int64_t max) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	if (getting) return ERR_BUSY;
	callbacks.getNumber = callback;
	minNum = min;
	maxNum = max;
	getLine(&_getLineNumberHandler);
	return ERR_NO;
}

/**
 * Prints a character as soon as the buffer is empty. Does not wait for getting
 * operations to complete.
 */
static inline void _printCharNB(char letter) {
	while (!(uartModule->IFG & EUSCI_A_IFG_TXIFG));
	uartModule->TXBUF = letter;
}

static inline void _printChar(char letter) {
	while (!(uartModule->IFG & EUSCI_A_IFG_TXIFG) || getting); // Block until done with any get operations.
	uartModule->TXBUF = letter;
}

error printChar(char letter) {
	if (!uartModule) return ERR_UNINITIALIZED;
	_printChar(letter);
	return ERR_NO;
}

static inline void _printNewlineNB(void) {
	_printCharNB('\n');
	_printCharNB('\r');
}

static inline void _printNewline(void) {
	_printChar('\n');
	_printChar('\r');
}

error printNewline(void) {
	if (!uartModule) return ERR_UNINITIALIZED;
	_printNewline();
	return ERR_NO;
}

static inline void _printString(char * string) {
	while (*string) _printChar(*(string++));
}

error printString(char * string) {
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	_printString(string);
	return ERR_NO;
}

error printArray(uint32_t len, char ** strArr) {
	uint32_t i = 0;
	char * str = NULL;
	if (!strArr) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	for (; i < len; i++) {
		str = strArr[i];
		if (!str) return ERR_NULL_PTR;
		printWrap(str, 0, 0, 0);
	}
	return ERR_NO;
}

/**
 * Prints a string and wraps at 80 chars.
 */
static inline void _printWrap(char * string, uint8_t lvl1Indent, uint8_t lvlNIndent, uint8_t lvl1CharsPrinted) {
	uint8_t i = 0, j = lvl1Indent + lvl1CharsPrinted;
	while (lvl1Indent--) _printChar(' ');
	for (; string[i] && j < 80; i++, j++) { // Printed first line
		if (string[i] == '\n') {
			j = 80;
		}
		else _printChar(string[i]);
	}
	_printNewline();
	while (string[i]) {
		for (j = lvlNIndent; j; j--) _printChar(' ');
		if (string[i] == ' ' || string[i] == '\n') i++;
		for (j = lvlNIndent; j < 80 && string[i]; i++, j++) {
			if (string[i] == '\n') {
				j = 80;
			}
			else _printChar(string[i]);
		}
		_printNewline();
	}
}


error printWrap(char * string, uint8_t lvl1Indent, uint8_t lvlNIndent, uint8_t lvl1CharsPrinted) {
	if (lvl1Indent > 79 || lvlNIndent > 79) return ERR_PARAM_OUT_OF_BOUNDS;
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	_printWrap(string, lvl1Indent, lvlNIndent, lvl1CharsPrinted);
	return ERR_NO;
}

error log(char * string) {
	// The number of logging operations active currently. Allows printing of log
	// break characters '<'/'>', '{'/'}', and '['/']' for nested logging events,
	// potentially caused by logs in interrupts.
	static uint8_t logLevel = 0;
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	pauseCommands();
	logLevel++;
	if (logLevel > 1) {
		_printChar(logBreakOpeners[(logLevel + 1) % 3]);
		_printNewline();
	}
	char num[6];
	uIntToStr(getHours(), num, 2);
	_printString(num);
	_printChar(':');
	uIntToStr(getMinutes(), num, 2);
	_printString(num);
	_printChar(':');
	uIntToStr(getSeconds(), num, 2);
	_printString(num);
	_printString(" - ");
	_printWrap(string, 0, 11, 11);
	if (logLevel > 1) _printChar(logBreakClosers[(logLevel + 1) % 3]);
	else readyForCommands(); // Only ready if exiting last logging level
	logLevel--;
	return ERR_NO;
}

static void _stopGettingLine(void) {
	getting = GETTING_NO;     // Stop getting line
	if (!callbacks.command) uartModule->IE &= ~EUSCI_A_IE_RXIE; // interrupt enabled by getLine
}

static void inline _print80NB(char * string, uint8_t charsPrinted) {
	while (*string && charsPrinted < 80) {
		_printCharNB(*(string++));
		charsPrinted++;
	}
	while (charsPrinted < 80) {
		_printCharNB(' ');
		charsPrinted++;
	}
}

static void _rxHandler() {
	static char * line = NULL;
	static uint32_t charsRead = 0;
	static uint32_t lineSize = 0;
	static error err = ERR_NO;

	switch (getting) {

	case GETTING_NO:
		if (callbacks.command) {
			if (!flags.readyForCommand) flags.unhandledCommand = 1;
			else {
				flags.unhandledCommand = 0;
				// Printable
				if (printable(lastRX)) _printChar(lastRX);
				_printNewline();
				(*callbacks.command)(lastRX);
			}
		}
		break;

	case GETTING_LINE:
		charsRead++;
		if (lastRX == '\r') {             // User is done
			_stopGettingLine();
			lastRX = '\0';                // Push null terminator
			_printNewlineNB();
		}
		else _printCharNB(lastRX);
		if (charsRead + 1 > lineSize) { // Ran out of room
			char * newLine = NULL;
			if (!lineSize) {
				newLine = malloc(1);    // The first char and a '\0'
			}
			else {
				newLine = realloc(line, lineSize * 2 + 1); // Try to get more
			}
			if (!newLine) {             // Crap...
				if (line) line[lineSize - 1] = '\0';
				_stopGettingLine();
				err = ERR_OUT_OF_MEM;
			}
			else {
				line = newLine;
				lineSize = lineSize * 2 + 1;
			}
		}
		if (line && err == ERR_NO) line[charsRead - 1] = lastRX;
		if (!getting) { // Done
			(*callbacks.getLine)(line, err);
			line = NULL;
			charsRead = 0;
			lineSize = 0;
			err = ERR_NO;
		}
		break;

	case GETTING_CHAR:
		getting = GETTING_NO;     // Stop getting char
		(*callbacks.getChar)(lastRX);
		if (!callbacks.command) uartModule->IE &= ~EUSCI_A_IE_RXIE; // interrupt enabled by getChar
		break;

	case GETTING_OPTION:
		_printCharNB('\r');
		_printCharNB('>');
		switch (lastRX) {
		case 'u': // Up
		case 'p': // Previous
		case 'j':
			if (currentOption > 0) currentOption--;
			_print80NB(options[currentOption], 1);
			break;
		case 'd': // Down
		case 'k':
		case 'n': // Next
			if (currentOption < numOptions - 1) currentOption++;
			_print80NB(options[currentOption], 1);
			break;
		case '\r':
			_print80NB("", 1); // Clear line by printing 80 spaces
			_printCharNB('\r');
			getting = GETTING_NO;
			(*callbacks.getOption)(currentOption);
			if (!callbacks.command) uartModule->IE &= ~EUSCI_A_IE_RXIE; // interrupt enabled by getOption
			break;
		}
		break;
	}
}

error readyForCommands(void) {
	if (!uartModule || !callbacks.command) return ERR_UNINITIALIZED;
	if (getting) return ERR_RACE_COND; // Locks up on print
	if (!flags.readyForCommand) {
		flags.readyForCommand = 1;
		_printChar('$');
		_printChar(' ');
	}
	if (flags.unhandledCommand) runAsync(&_rxHandler);
	return ERR_NO;
}

error pauseCommands(void) {
	if (!uartModule) return ERR_UNINITIALIZED;
	flags.readyForCommand = 0;
	if (getting) return ERR_RACE_COND; // Locks up on print
	_printChar('\r');
	return ERR_NO;
}

void eUSCIUARTHandler(void) {
	if (uartModule->IFG & EUSCI_A_IFG_RXIFG) {
		lastRX = uartModule->RXBUF; // Clears IFG
		runAsync(&_rxHandler);
	}
}
