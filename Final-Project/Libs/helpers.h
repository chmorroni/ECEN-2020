#include <stdint.h>
#include "error.h"

#ifndef HELPERS
#define HELPERS

#ifndef NULL
#define NULL (0)
#endif
#define FLAG(name)    uint8_t name: 1 // Bit field in struct
#define REG(name, n)  int8_t name: n // N-bit signed register bit field
#define UREG(name, n) uint8_t name: n // N-bit unsigned register bit field

typedef enum {
	END_BIG,
	END_LITTLE,
	END_JUST_PRINT_THE_THING
} endianness;

inline int32_t strLen(char * str);
inline uint8_t printable(char character);
inline uint8_t numeric(char character);
error uIntToStr(uint64_t num, char * string, uint8_t minWidth);
error intToStr(int64_t num, char * string, uint8_t minWidth);

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
error strToInt64(char * string, int64_t * container);

#endif
