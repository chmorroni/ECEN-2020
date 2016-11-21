#ifndef MSP_ASYNC
#define MSP_ASYNC

#include <stdint.h>
#include "msp.h"
#include "helpers.h"

typedef void (* asyncFuncPtr)(void);

void initAsync(void);

void runAsync(asyncFuncPtr);

#endif
