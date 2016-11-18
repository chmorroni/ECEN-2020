#include "logging.h"
#include "serial.h"
#define BUFF_TYPE uint8_t
#include "buffer.h"
#include "timers.h"
#include "helpers.h"

static EUSCI_A_Type * uartModule = NULL;
static Buff uartBuff = {0};
static commandFuncPtr commandHandler = NULL;
static getLinePtr getLineHandler = NULL;

#define GETTING_LINE (0x1)
#define LOGGING      (0x2)
static uint8_t flags = 0;


void eUSCIUARTHandler(void) {
	uint8_t data;
	if (uartModule->IFG & EUSCI_A_IFG_RXIFG) {
		data = uartModule->RXBUF;
		// TODO: implement these shenanigans with PendSV interrupt.
		if (flags & GETTING_LINE) {
			static char * line = NULL;
			static uint32_t charsRead = 0;
			static uint32_t lineSize = 0;
			static error err = ERR_NO;
			charsRead++;
			if (data == '\r') {             // User is done
				flags &= ~GETTING_LINE;     // Stop getting line
				data = '\0';                // Push null terminator
			}
			else printCharIS(data);
			if (charsRead + 1 > lineSize) { // Ran out of room
				char * newLine = NULL;
				if (!lineSize) {
					lineSize = 2;
					newLine = malloc(lineSize); // The first char and a '\0'
				}
				else {
					lineSize = lineSize * 2;
					newLine = realloc(line, lineSize); // Try to get more
				}
				if (!newLine) {             // Crap...
					flags &= ~GETTING_LINE;
					err = ERR_OUT_OF_MEM;
				}
				else line = newLine;
			}
			if (line && err == ERR_NO) line[charsRead - 1] = data;
			if (!(flags & GETTING_LINE)) { // Done
				getLineHandler(line, err);
				line = NULL;
				charsRead = 0;
				lineSize = 0;
				err = ERR_NO;
			}
		}
		else if (commandHandler) {
			printCharIS(data);
			printCharIS('\n');
			printCharIS('\r');
			(*commandHandler)(data);
		}
	}
	if (uartModule->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&uartBuff, &data) == ERR_NO) uartModule->TXBUF = data;
		else uartModule->IE &= ~EUSCI_A_IE_TXIE;
	}
}

inline void printChar(char letter) {
	while(!(flags & GETTING_LINE) && addToBuff(&uartBuff, letter) != ERR_NO);
	uartModule->IE |= EUSCI_A_IE_TXIE;
}

inline void printCharIS(char letter) {
	while (!(flags & GETTING_LINE) && !(uartModule->IFG & EUSCI_A_IFG_TXIFG));
	uartModule->TXBUF = letter;
}

error configLogging(EUSCI_A_Type * module, uint32_t baud, uint32_t size) {
	error err = ERR_NO;
	if (!module) return ERR_NULL_PTR;
	err = configUART(module, baud);
	if (err != ERR_NO) return err;
	uartModule = module;
	err = initBuff(&uartBuff, size);
	if (err != ERR_NO) return err;
	initRTC();
	return err;
}

error startLogging(IRQn_Type uartIRQ) {
	error err = ERR_NO;
	if (!uartModule) return ERR_UNINITIALIZED;
	err = startUART(uartModule);
	uartModule->IE |= EUSCI_A_IE_TXIE;
	NVIC_EnableIRQ(uartIRQ);
	return err;
}

error enableCommands(commandFuncPtr callback) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	uartModule->IE |= EUSCI_A_IE_RXIE;
	commandHandler = callback;
	return ERR_NO;
}

error getLine(getLinePtr callback) {
	if (!callback) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	getLineHandler = callback;
	flags |= GETTING_LINE;
	return ERR_NO;
}

error printString(char * string) {
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	while (*string) printChar(*(string++));
	return ERR_NO;
}

error printStringIS(char * string) {
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	while (*string) printCharIS(*(string++));
	return ERR_NO;
}

error log(char * string) {
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	char num[6];
	uint16_t hours = 0, minutes = 0, seconds = 0;
	getTime(&hours, &minutes, &seconds);
	uIntToStr(hours, num, 2);
	printString(num);
	printChar(':');
	uIntToStr(minutes, num, 2);
	printString(num);
	printChar(':');
	uIntToStr(seconds, num, 2);
	printString(num);
	printString(" - ");
	printString(string);
	printChar('\n');
	printChar('\r');
	return ERR_NO;
}

error logIS(char * string) {
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	char num[6];
	uint16_t hours = 0, minutes = 0, seconds = 0;
	getTime(&hours, &minutes, &seconds);
	uIntToStr(hours, num, 2);
	printCharIS('\n');
	printCharIS('\r');
	printStringIS(num);
	printCharIS(':');
	uIntToStr(minutes, num, 2);
	printStringIS(num);
	printCharIS(':');
	uIntToStr(seconds, num, 2);
	printStringIS(num);
	printStringIS(" - ");
	printStringIS(string);
	printCharIS('\n');
	printCharIS('\r');
	return ERR_NO;
}

error printNewline(void) {
	if (!uartModule) return ERR_UNINITIALIZED;
	printChar('\n');
	printChar('\r');
	return ERR_NO;
}

error printNewlineIS(void) {
	if (!uartModule) return ERR_UNINITIALIZED;
	printCharIS('\n');
	printCharIS('\r');
	return ERR_NO;
}
