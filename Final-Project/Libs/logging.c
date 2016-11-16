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
			if (data == '\r') {
				flags &= ~GETTING_LINE;
				data = '\0';
			}
			if (charsRead + 1 > lineSize) {
				char * newLine = realloc(line, lineSize * 2 + 1);
				if (!newLine) {
					flags &= ~GETTING_LINE;
					err = ERR_OUT_OF_MEM;
				}
				else line = newLine;
			}
			if (line) line[charsRead++] = data;
			if (!(flags & GETTING_LINE)) { // Done
				getLineHandler(line, err);
				line = NULL;
				charsRead = 0;
				lineSize = 0;
				err = ERR_NO;
			}
		}
		else if (commandHandler) (*commandHandler)(uartModule->RXBUF);
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
	uInt16ToStr(hours, num, 2);
	printChar('\n');
	printChar('\r');
	printString(num);
	printChar(':');
	uInt16ToStr(minutes, num, 2);
	printString(num);
	printChar(':');
	uInt16ToStr(seconds, num, 2);
	printString(num);
	printString(" - ");
	printString(string);
	return ERR_NO;
}

error logIS(char * string) {
	if (!string) return ERR_NULL_PTR;
	if (!uartModule) return ERR_UNINITIALIZED;
	char num[6];
	uint16_t hours = 0, minutes = 0, seconds = 0;
	getTime(&hours, &minutes, &seconds);
	uInt16ToStr(hours, num, 2);
	printCharIS('\n');
	printCharIS('\r');
	printStringIS(num);
	printCharIS(':');
	uInt16ToStr(minutes, num, 2);
	printStringIS(num);
	printCharIS(':');
	uInt16ToStr(seconds, num, 2);
	printStringIS(num);
	printStringIS(" - ");
	printStringIS(string);
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
