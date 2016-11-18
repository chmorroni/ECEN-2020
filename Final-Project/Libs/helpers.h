#include <stdint.h>
#include "error.h"

#ifndef HELPERS
#define HELPERS

#ifndef NULL
#define NULL (0)
#endif

typedef enum {
	END_BIG,
	END_LITTLE,
	END_JUST_PRINT_THE_THING
} endianness;

inline int32_t strLen(char * str);
error uIntToStr(uint64_t num, char * string, uint8_t minWidth);
/**
 * String should be at least 2 * bytesToPrint + 3 ('0' + 'x' + 2 chars per byte + '\0'
 */
error bytesToHexStr(uint8_t * bytes, char * string, uint32_t bytesToPrint, endianness end);

/**
 *
 */
error uInt8ToHexStr(uint8_t num, char * string);
error uInt16ToHexStr(uint16_t num, char * string);
error uInt32ToHexStr(uint32_t num, char * string);
error uInt64ToHexStr(uint64_t num, char * string);

error strToUInt32(char * string, uint32_t * container);
error strToUInt64(char * string, uint64_t * container);

#endif
