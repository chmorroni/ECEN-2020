#include "logging.h"
#include "serial.h"
#define BUFF_TYPE uint8_t
#include "buffer.h"
#include "timers.h"
#include "helpers.h"

static EUSCI_A_Type * uartModule = NULL;
static LLBuff uartBuff = {0};
static commandFuncPtr commandHandler = NULL;
static getLinePtr getLineHandler = NULL;
static uint8_t gettingLine = 0;
static uint8_t newData = 0;
static char lastRead = '\0';

void eUSCIUARTHandler(void) {
	uint8_t data;
	if (uartModule->IFG & EUSCI_A_IFG_RXIFG) {
		data = uartModule->RXBUF;
		// TODO: implement these shenanigans with PendSV interrupt.
		if (gettingLine) {
			static char * line = NULL;
			static uint32_t charsRead = 0;
			static uint32_t lineSize = 0;
			static error err = NO_ERR;

			if (data == '\r') {
				gettingLine = 0;
				data = '\0';
			}
			if (charsRead + 1 > lineSize) {
				char * newLine = realloc(line, lineSize * 2 + 1);
				if (!newLine) {
					gettingLine = 0;
					err = ERR_OUT_OF_MEM;
				}
				else line = newLine;
			}
			if (line) line[charsRead++] = data;
			if (!gettingLine) { // Done
				getLineHandler(line, err);
				line = NULL;
				charsRead = 0;
				lineSize = 0;
				err = NO_ERR;
			}
		}
		else if (commandHandler) (*commandHandler)(uartModule->RXBUF);
	}
	if (uartModule->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromLLBuff(&uartTXBuff, &data) == ERR_NO) uartModule->TXBUF = data;
		else uartModule->IE &= ~EUSCI_A_IE_TXIE;
	}
}

static inline void printChar(char letter) {
	while(!gettingLine && addToLLBuff(&uartBuff, letter) != ERR_NO);
	uartModule->IE |= EUSCI_A_IE_TXIE;
}

static inline void printCharIS(char letter) {
	while (!(uartModule->IFG & EUSCI_A_IFG_TXIFG));
	uartModule->TXBUF = letter;
}

error configLogging(EUSCI_A_Type * module, uint32_t baud, uint32_t size) {
	error err = ERR_NO;
	if (!module) return ERR_NULL_PTR;
	err = configUART(module, baud);
	if (err != ERR_NO) return err;
	uartModule = module;
	err = initLLBuff(&uartBuff, size);
	if (err != ERR_NO) return err;
	initRTC();
	return err;
}

error startLogging(void) {
	if (!module) return ERR_UNINITIALIZED;
	return startUART(module);
}

error enableCommands(commandFuncPtr callback) {
	if (!callback) return ERR_NULL_PTR;
	commandHandler = callback;
	return ERR_NO;
}

error getLine(getLinePtr callback) {
	if (!callback) return ERR_NULL_PTR;
	getLineHandler = callback;
	gettingLine = 1;
	return ERR_NO;
}

error printString(char * string) {
	while (*string) printByte(*(string++));
}

error printStringIS(char * string) {
	while (*string) printByteIS(*(string++));
}

error log(char * string) {
	char num[6];
	uint16_t hours = 0, minutes = 0, seconds = 0;
	getTime(&hours, &minutes, &hours);
	uInt16ToStr(hours, num);
	printString(num);
	printChar(':');
	uInt16ToStr(minutes, num);
	printString(num);
	printChar(':');
	uInt16ToStr(seconds, num);
	printString(num);
	printString(" - ");
	printString(string);
	printChar('\n');
	printChar('\r');
}

error logIS(char * string) {
	char num[6];
	uint16_t hours = 0, minutes = 0, seconds = 0;
	getTime(&hours, &minutes, &hours);
	uInt16ToStr(hours, num);
	printStringIS(num);
	printCharIS(':');
	uInt16ToStr(minutes, num);
	printStringIS(num);
	printCharIS(':');
	uInt16ToStr(seconds, num);
	printStringIS(num);
	printStringIS(" - ");
	printStringIS(string);
	printCharIS('\n');
	printCharIS('\r');
}
