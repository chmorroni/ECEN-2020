#include <stdint.h>
#include "error.h"

int32_t strLen(char * str);
error uInt16ToStr(uint16_t num, char * string);
error uInt32ToStr(uint32_t num, char * string);
error uInt64ToStr(uint64_t num, char * string);
error strToUInt32(char * string, uint32_t * container);
error strToUInt64(char * string, uint64_t * container);
