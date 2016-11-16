#include "helpers.h"
/**
 * @desc includes null terminator
 */
int32_t strLen(char * str) {
	int32_t len = 1;
	if (!str) return -1;
	for (len = 1; *str++; len++);
	return len;
}

error uInt16ToStr(uint16_t num, char * string, uint8_t minWidth) {
    uint8_t i = 0;         // If number is zero, one char must still be transmitted: '0'
    uint8_t firstDigit;    // Used to diminish number of modulo operations
    uint16_t numCpy = num; // Don’t want to change num just yet
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
error uInt32ToStr(uint32_t num, char * string, uint8_t minWidth) {
    uint8_t i = 0;         // If number is zero, one char must still be transmitted: '0'
    uint8_t firstDigit;    // Used to diminish number of modulo operations
    uint32_t numCpy = num; // Don’t want to change num just yet
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
error uInt64ToStr(uint64_t num, char * string, uint8_t minWidth) {
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

error strToUInt32(char * string, uint32_t * container) {
    uint8_t magnitude = 1; // scalling factor of current char
    uint32_t i = 0;
    uint32_t num = 0;
    uint32_t oldNum = 0;
	if (!string || !container) return ERR_NULL_PTR;
	while (string[i]) {
		if (string[i] < '0' || string[i] > '9') return ERR_NAN;
		num += magnitude * (string[i] - 0x30);
		magnitude *= 10;
		if (num < oldNum) return ERR_OVERFLOW; // Unsigned overflow, defined
		oldNum = num;
	}
	return ERR_NO;
}

error strToUInt64(char * string, uint64_t * container) {
    uint8_t magnitude = 1; // scalling factor of current char
    uint32_t i = 0;
    uint64_t num = 0;
    uint64_t oldNum = 0;
	if (!string || !container) return ERR_NULL_PTR;
	while (string[i]) {
		if (string[i] < '0' || string[i] > '9') return ERR_NAN;
		num += magnitude * (string[i] - 0x30);
		magnitude *= 10;
		if (num < oldNum) return ERR_OVERFLOW; // Unsigned overflow, defined
		oldNum = num;
	}
	return ERR_NO;
}
