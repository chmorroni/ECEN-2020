#ifndef MSP_LOG
#define MSP_LOG

#include <stdint.h>
#include "msp.h"
#include "error.h"

typedef void (* commandFuncPtr)(uint8_t received);
typedef void (* getLinePtr)(char * received, error * err);

void eUSCIUARTHandler(void);

/**
 * @desc Enables UART on the specified module and sets up the buffer.
 */
error configLogging(EUSCI_A_Type * module, uint32_t baud, uint32_t size);
error startLogging(void);
error enableCommands(commandCallback callback);
error getLine(getLineCallback callback);
error log(char * string);
error logIS(char * string);
error printString(char * string);
error printStringIS(char * string);


#endif
