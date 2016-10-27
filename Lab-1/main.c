
#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>

#define p1out (*((uint8_t *) 0x40004C02))		// P1OUT address
#define p2out (*((uint8_t *) 0x40004C03))		// P2OUT address
#define p1dir (*((uint8_t *) 0x40004C04))		// P1DIR address
#define p2dir (*((uint8_t *) 0x40004C05))		// P2DIR address

#define P0DELAY 30000		// Iterations for problem 0 (part 6.1 in the lab)
#define P5DELAY 10000		// Iterations for problem 5; 1, 10, 100, 1000, 10000
#define P6DELAY 13750		// Iterations for problem 6; times 2 for total delay
#define P7DELAY 27500		// Total iterations for problem 7
#define DUTYCYCLE 50			// Duty cycle for problem 7
#define PERCENT 100			// Constant for division in problem 7

#define PIN0 0x01			// Bit mask for pin 0
#define PIN1 0x02			// Bit mask for pin 1
#define PIN2 0x04			// Bit mask for pin 2
#define PIN5 0x20			// Bit mask for pin 5
#define PIN7 0x80			// Bit mask for pin 7

#undef PROBLEM0
#undef PROBLEM1
#undef PROBLEM2
#undef PROBLEM3
#undef PROBLEM4
#define PROBLEM5
#undef PROBLEM6
#undef PROBLEM7

void squareWave(uint8_t dutyCycle);

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;			// Stop watchdog timer
    char * row = "%15s |%5d\n";			// Format to print the table in
    p1out &= ~PIN0;					// Turn off red LED
    p2out &= (PIN0 | PIN1 | PIN2);			// Turn off RGB LED
    uint32_t i = 0;					// for counter

#ifdef PROBLEM0	// This is the LED blink code from section 6.1
    uint32_t count = 0;
    p1dir = PIN0;

    while(1){
    	count++;
    	p1out ^= PIN0;
    	for(i = P0DELAY; i > 0; i--);
    	printf("Testing %d\n", count);
    }
#endif

#ifdef PROBLEM1
    printf("\n");
    printf("           type | size\n"); 	// Print header
    printf("\n");
    printf(row, "char", sizeof(char));
    printf(row, "int", sizeof(int));
    printf(row, "short", sizeof(short));
    printf(row, "long", sizeof(long));
    printf(row, "float", sizeof(float));
    printf(row, "double", sizeof(double));
    printf(row, "long double", sizeof(long double));
    printf("\n");
#endif

#ifdef PROBLEM2
    printf("\n");
    printf("           type | size\n"); 	// Print header
    printf("\n");
    printf(row, "int8_t", sizeof(int8_t));
    printf(row, "int16_t", sizeof(int16_t));
    printf(row, "int32_t", sizeof(int32_t));
    printf(row, "uint8_t", sizeof(uint8_t));
    printf(row, "uint16_t", sizeof(uint16_t));
    printf(row, "uint32_t", sizeof(uint32_t));
    printf("\n");
#endif

#ifdef PROBLEM3
    printf("\n");
    printf("           type | size\n"); 	// Print header
    printf("\n");
    printf(row, "int8_t *", sizeof(int8_t *));
    printf(row, "char *", sizeof(char *));
    printf(row, "int *", sizeof(int *));
    printf(row, "unsigned int *", sizeof(unsigned int *));
    printf(row, "uint16_t *", sizeof(uint16_t *));
    printf("\n");
#endif

#ifdef PROBLEM4
    p1dir |= PIN0;		// Set pin 1.0 to output (red LED)
    p2dir |= PIN1;		// Set pin 2.2 to output (green on RGB LED)
    p1out |= PIN0;		// Set pin 1.0 to high (red LED)
    p2out |= PIN1;		// Set pin 2.2 to high (green on RGB LED)
#endif

#ifdef PROBLEM5
    p2dir = PIN5;					// Set pin 2.5 to output
    while(1){
       p2out = PIN5;					// Set pin 2.5 high
    	for(i = 0; i < P5DELAY; i++);		// Delay
    	p2out = 0;					// Set pin 2.5 low
    }

#endif

#ifdef PROBLEM6
    p1dir = PIN7;					// Set pin 1.7 to output
    while(1){
    	p1out = PIN7;					// Set pin 1.7 high
    	for(i = 0; i < P6DELAY; i++);		// Delay
    	p1out = 0;					// Set pin 1.7 high
    	for(i = 0; i < P6DELAY; i++);		// Delay
    }
#endif

#ifdef PROBLEM7
    squareWave(DUTYCYCLE);				// Generate square wave
#endif

    while(1);						// Ensure main doesn't exit
}

void squareWave(uint8_t dutyCycle){
	p1dir = PIN7;							// Set pin 1.7 to output
	uint32_t highDelay = P7DELAY * dutyCycle / PERCENT;	// Find time spent high
	uint32_t lowDelay = P7DELAY - highDelay;			// Find time spent low
	uint32_t i;
	while(1){
		p1out = PIN7;						// Set pin 1.7 high
		for(i = 0; i < highDelay; i++);			// Delay
		p1out = 0;						// Set pin 1.7 low
		for(i = 0; i < lowDelay; i++);			// Delay
	}
}
