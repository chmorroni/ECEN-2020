#include "helpers.h"
/**
 * @desc includes null terminator
 */
inline int32_t strLen(char * str) {
	int32_t len = 1;
	if (!str) return -1;
	for (len = 1; *str++; len++);
	return len;
}

error uIntToStr(uint64_t num, char * string, uint8_t minWidth) {
    uint8_t i = 0;         // If number is zero, one char must still be transmitted: '0'
    uint8_t firstDigit;    // Used to diminish number of modulo operations
    uint64_t numCpy = num; // Don’t want to change num just yet
    if (!string) return ERR_NULL_PTR;
    // Since we are writing the number from right to left, the top of the stack must
    // contain the one’s place. We do this by starting at the end. Therefore, we need to
    // know how long the output string will be:
    for (i = 1; numCpy = (numCpy - (numCpy % 10)) / 10; i++);
    if (i < minWidth) i = minWidth;    // i now holds index of '\0'
    string[i] = '\0';
    do {
        i--;                           // Move down string
        firstDigit = num % 10;         // Get number in ones place
        string[i] = firstDigit + 0x30; // 0x30 is ASCII offset to when digits start
        num -= firstDigit;             // Remove possibility of rounding issues
        num /= 10;                     // Shift numbers down one power of 10
    } while (i);
    return ERR_NO;
}

static inline char nibbleToChar(uint8_t nibble) {
	char offset = nibble < 0xA ? '0' : 'A' - 0xA;
	return nibble > 0xF ? 'U' : nibble + offset;
}

error bytesToHexStr(uint8_t * bytes, char * string, uint32_t bytesToPrint, endianness end) {
	uint32_t i, j; // Used for loops
    if (!string || !bytes) return ERR_NULL_PTR;
    if (!bytesToPrint) return ERR_PARAM_OUT_OF_BOUNDS;
    string[0] = '0';
    string[1] = 'x';
    switch (end) {
    case END_JUST_PRINT_THE_THING: // Fall through
    case END_BIG:
    	for (i = 0; i < bytesToPrint; i++) { // MSB comes first, go up array
    	    string[i*2 + 2] = nibbleToChar((bytes[i] & 0xF0) >> 4);
    	    string[i*2 + 3] = nibbleToChar(bytes[i] & 0xF);
    	}
    	break;
    case END_LITTLE:
    	for (i = 0, j = bytesToPrint - 1; i < bytesToPrint; i++, j--) { // MSB comes first, go up array
    	    string[i*2 + 2] = nibbleToChar((bytes[j] & 0xF0) >> 4);
    	    string[i*2 + 3] = nibbleToChar(bytes[j] & 0xF);
    	}
    	break;
    }
    string[bytesToPrint*2 + 2] = '\0';
    return ERR_NO;
}

error uInt8ToHexStr(uint8_t num, char * string) {
	return bytesToHexStr(&num, string, sizeof (uint8_t), END_LITTLE);
}

error uInt16ToHexStr(uint16_t num, char * string) {
	return bytesToHexStr((uint8_t *) &num, string, sizeof (uint16_t), END_LITTLE);
}

error uInt32ToHexStr(uint32_t num, char * string) {
	return bytesToHexStr((uint8_t *) &num, string, sizeof (uint32_t), END_LITTLE);
}

error uInt64ToHexStr(uint64_t num, char * string) {
	return bytesToHexStr((uint8_t *) &num, string, sizeof (uint64_t), END_LITTLE);
}

error strToUInt32(char * string, uint32_t * container) {
	if (!string || !container) return ERR_NULL_PTR;
    uint32_t magnitude = 1; // Scaling factor of current char
    int32_t len = strLen(string);
    int32_t i = 0;
    uint32_t num = 0;
    uint32_t oldNum = 0;
    if (len == 1) return ERR_NAN; // Empty string
	for (i = len - 2; i >= 0; i--) {
		if (string[i] < '0' || string[i] > '9') return ERR_NAN;
		num += magnitude * (string[i] - 0x30);
		magnitude *= 10;
		if (num < oldNum) return ERR_OVERFLOW; // Unsigned overflow, defined
		oldNum = num;
	}
	*container = num;
	return ERR_NO;
}

error strToUInt64(char * string, uint64_t * container) {
	if (!string || !container) return ERR_NULL_PTR;
    uint64_t magnitude = 1; // Scaling factor of current char
    int32_t len = strLen(string);
    int32_t i = 0;
    uint64_t num = 0;
    uint64_t oldNum = 0;
    if (len == 1) return ERR_NAN; // Empty string
	for (i = len - 2; i >= 0; i--) {
		if (string[i] < '0' || string[i] > '9') return ERR_NAN;
		num += magnitude * (string[i] - 0x30);
		magnitude *= 10;
		if (num < oldNum) return ERR_OVERFLOW; // Unsigned overflow, defined
		oldNum = num;
	}
	*container = num;
	return ERR_NO;
}
