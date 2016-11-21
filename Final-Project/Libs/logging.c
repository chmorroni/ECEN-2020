#include "logging.h"
#include "serial.h"
#define BUFF_TYPE uint8_t
#include "buffer.h"
#include "timers.h"
#include "helpers.h"
#include "async.h"

static EUSCI_A_Type * uartModule = NULL;
static Buff uartBuff = {0};
static commandFuncPtr commandHandler = NULL;
static getCharPtr getCharHandler = NULL;
static getLinePtr getLineHandler = NULL;
#define GETTING_LINE (0x4)
#define GETTING_CHAR (0x2)
#define TIMESTAMPS   (0x1)
static uint8_t flags = 0;
static uint8_t lastRX = 0;
// The number of logging operations active currently. Allows printing of log
// break characters '<'/'>', '{'/'}', and '['/']' for nested logging events,
// potentially caused by logs in interrupts.
static uint8_t logLevel = 0;
static char logBreakOpeners[] = {'<', '{', '['};
static char logBreakClosers[] = {'>', '}', ']'};

static inline void _printCharAS(char letter) {
	if (uartModule->IFG & EUSCI_A_IFG_TXIFG && buffEmpty(&uartBuff)) uartModule->TXBUF = letter;
	else {
		while(addToBuff(&uartBuff, letter) != ERR_NO);
		uartModule->IE |= EUSCI_A_IE_TXIE;
	}
}

static inline void _printCharIS(char letter) {
	while (!(uartModule->IFG & EUSCI_A_IFG_TXIFG));
	uartModule->TXBUF = letter;
}

static inline void _printNewlineIS(void) {
	_printCharIS('\n');
	_printCharIS('\r');
}

error configLogging(EUSCI_A_Type * module, uint32_t baud, uint32_t size) {
	error err = ERR_NO;
	if (!module) return ERR_NULL_PTR;
	err = configUART(module, baud);
	if (err != ERR_NO) return err;
	uartModule = module;
	err = initBuff(&uartBuff, size);
	if (err != ERR_NO) return err;
	initAsync();
	return err;
}

error startLogging(IRQn_Type uartIRQ) {
	error err = ERR_NO;
	if (!uartModule) return ERR_UNINITIALIZED;
	err = startUART(uartModule);
	uartModule->IE |= EUSCI_A_IE_TXIE;
	NVIC_EnableIRQ(uartIRQ);
	initRTC();
	return err;
}

error enableCommands(commandFuncPtr callback) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	uartModule->IE |= EUSCI_A_IE_RXIE;
	commandHandler = callback;
	return ERR_NO;
}

error getChar(getCharPtr callback) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	if (flags & (GETTING_CHAR | GETTING_LINE)) return ERR_BUSY;
	getCharHandler = callback;
	uartModule->IE |= EUSCI_A_IE_RXIE;
	flags |= GETTING_CHAR;
	return ERR_NO;
}

error getLine(getLinePtr callback) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	if (flags & (GETTING_CHAR | GETTING_LINE)) return ERR_BUSY;
	getLineHandler = callback;
	uartModule->IE |= EUSCI_A_IE_RXIE;
	flags |= GETTING_LINE;
	return ERR_NO;
}

static inline void _printChar(char letter, sendMode mode) {
	while(flags & (GETTING_LINE | GETTING_CHAR)); // Block while getline is active, don't want garbled I/O
	switch (mode) {
	case SEND_ASYNC:
		_printCharAS(letter);
		break;
	case SEND_INTERRUPT_SAFE:
		_printCharIS(letter);
		break;
	}
}

error printChar(char letter, sendMode mode) {
	if (!uartModule) return ERR_UNINITIALIZED;
	_printChar(letter, mode);
	return ERR_NO;
}

static inline void _printNewline(sendMode mode) {
	_printChar('\n', mode);
	_printChar('\r', mode);
}

error printNewline(sendMode mode) {
	if (!uartModule) return ERR_UNINITIALIZED;
	_printNewline(mode);
	return ERR_NO;
}

static inline void _printString(char * string, sendMode mode) {
	while (*string) _printChar(*(string++), mode);
}

error printString(char * string, sendMode mode) {
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	while (*string) printChar(*(string++), mode);
	return ERR_NO;
}

static inline void _printWrap(char * string, uint8_t lvl1Indent, uint8_t lvlNIndent, uint8_t lvl1CharsPrinted, sendMode mode) {
	uint8_t i = 0, j = lvl1Indent + lvl1CharsPrinted;
	while (lvl1Indent--) _printChar(' ', mode);
	for (; string[i] && j < 80; i++, j++) {
		if (string[i] == '\n') {
			j = 80;
		}
		else _printChar(string[i], mode);
	}
	_printNewline(mode);
	while (string[i]) { // Printed first line
		for (j = lvlNIndent; j; j--) _printChar(' ', mode);
		if (string[i] == ' ' || string[i] == '\n') i++;
		for (j = lvlNIndent; j < 80 && string[i]; i++, j++) {
			if (string[i] == '\n') {
				j = 80;
			}
			else _printChar(string[i], mode);
		}
		_printNewline(mode);
	}
}


error printWrap(char * string, uint8_t lvl1Indent, uint8_t lvlNIndent, uint8_t lvl1CharsPrinted, sendMode mode) {
	if (lvl1Indent > 79 || lvlNIndent > 79) return ERR_PARAM_OUT_OF_BOUNDS;
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	_printWrap(string, lvl1Indent, lvlNIndent, lvl1CharsPrinted, mode);
	return ERR_NO;
}

error log(char * string, sendMode mode) {
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	logLevel++;
	if (logLevel > 1) { // TODO: Check if in interrupt (actually)
		_printChar(logBreakOpeners[(logLevel + 1) % 3], SEND_INTERRUPT_SAFE);
		_printNewline(SEND_INTERRUPT_SAFE);
		if (mode == SEND_ASYNC) {
			_printString("Use SEND_INTERRUPT_SAFE in interrupts!", SEND_INTERRUPT_SAFE);
			_printNewline(SEND_INTERRUPT_SAFE);
			_printChar(logBreakClosers[(logLevel + 1) % 3], SEND_INTERRUPT_SAFE);
			logLevel--;
			return ERR_RACE_COND; // Using this mode would cause a lock up if the buffer fills.
		}
	}
	if (flags & TIMESTAMPS) {
		char num[6];
		uIntToStr(getHours(), num, 2);
		_printString(num, mode);
		_printChar(':', mode);
		uIntToStr(getMinutes(), num, 2);
		_printString(num, mode);
		_printChar(':', mode);
		uIntToStr(getSeconds(), num, 2);
		_printString(num, mode);
		_printString(" - ", mode);
	}
	_printWrap(string, 0, flags & TIMESTAMPS ? 11 : 0, 11, mode);
	if (logLevel > 1) _printChar(logBreakClosers[(logLevel + 1) % 3], mode);
	_printNewline(mode);
	logLevel--;
	return ERR_NO;
}

void enableTimestamps(void) {
	flags |= TIMESTAMPS;
}

void disableTimestamps(void) {
	flags &= ~TIMESTAMPS;
}

static void _stopGettingLine(void) {
	flags &= ~GETTING_LINE;     // Stop getting line
	if (!commandHandler) uartModule->IE &= ~EUSCI_A_IE_RXIE; // interrupt enabled by getLine
}

static void _rxHandler() {
	if (flags & GETTING_LINE) {
		static char * line = NULL;
		static uint32_t charsRead = 0;
		static uint32_t lineSize = 0;
		static error err = ERR_NO;
		charsRead++;
		if (lastRX == '\r') {             // User is done
			_stopGettingLine();
			lastRX = '\0';                // Push null terminator
		}
		else _printCharIS(lastRX);
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
		if (!(flags & GETTING_LINE)) { // Done
			getLineHandler(line, err);
			getLineHandler = NULL;
			line = NULL;
			charsRead = 0;
			lineSize = 0;
			err = ERR_NO;
		}
	}
	else if (flags & GETTING_CHAR) {
		flags &= ~GETTING_CHAR;     // Stop getting line
		getCharHandler(lastRX);
		getCharHandler = NULL;
		if (!commandHandler) uartModule->IE &= ~EUSCI_A_IE_RXIE; // interrupt enabled by getChar
	}
	else if (commandHandler) {
		_printCharIS('$');
		_printCharIS(' ');
		// Printable
		if (lastRX >= 0x20 && lastRX <= 0x7D) _printCharIS(lastRX);
		_printNewlineIS();
		(*commandHandler)(lastRX);
	}
}

void eUSCIUARTHandler(void) {
	uint8_t data;
	if (uartModule->IFG & EUSCI_A_IFG_RXIFG) {
		lastRX = uartModule->RXBUF;
		runAsync(&_rxHandler);
	}
	if (uartModule->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&uartBuff, &data) == ERR_NO && !(flags & GETTING_LINE)) uartModule->TXBUF = data;
		else uartModule->IE &= ~EUSCI_A_IE_TXIE;
	}
}

