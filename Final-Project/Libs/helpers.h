#include <stdint.h>
#include "error.h"

#ifndef NULL
#define NULL (0)
#endif

int32_t strLen(char * str);
error uInt16ToStr(uint16_t num, char * string, uint8_t minWidth);
error uInt32ToStr(uint32_t num, char * string, uint8_t minWidth);
error uInt64ToStr(uint64_t num, char * string, uint8_t minWidth);
error strToUInt32(char * string, uint32_t * container);
error strToUInt64(char * string, uint64_t * container);
