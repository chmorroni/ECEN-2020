#include "helpers.h"

void itoa(uint32_t num, char * string) {
    uint8_t i = 1;         // If number is zero, one char must still be transmitted: '0'
    uint8_t firstDigit;    // Used to diminish number of modulo operations
    uint32_t numCpy = num; // Don’t want to change num just yet

    // Since we are writing the number from right to left, the top of the stack must
    // contain the one’s place. We do this by starting at the end. Therefore, we need to
    // know how long the output string will be:
    while (numCpy = (numCpy - (numCpy % 10)) / 10) i++; // i now holds index of '\0'
    string[i] = '\0';
    do {
        i--;                           // Move down string
        firstDigit = num % 10;         // Get number in ones place
        string[i] = firstDigit + 0x30; // 0x30 is ASCII offset to when digits start
        num -= firstDigit;             // Remove possibility of rounding issues
        num /= 10;                     // Shift numbers down one power of 10
    } while (i);
}

void ulltoa(uint64_t num, char * string) {
    uint8_t i = 1;         // If number is zero, one char must still be transmitted: '0'
    uint8_t firstDigit;    // Used to diminish number of modulo operations
    uint64_t numCpy = num; // Don’t want to change num just yet

    // Since we are writing the number from right to left, the top of the stack must
    // contain the one’s place. We do this by starting at the end. Therefore, we need to
    // know how long the output string will be:
    while (numCpy = (numCpy - (numCpy % 10)) / 10) i++; // i now holds index of '\0'
    string[i] = '\0';
    do {
        i--;                           // Move down string
        firstDigit = num % 10;         // Get number in ones place
        string[i] = firstDigit + 0x30; // 0x30 is ASCII offset to when digits start
        num -= firstDigit;             // Remove possibility of rounding issues
        num /= 10;                     // Shift numbers down one power of 10
    } while (i);
}
