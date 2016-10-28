#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include "Libs/uart.h"
#include "Libs/buffer_fifo.h"
#include "core_cm4.h"

#undef PB_1
#undef PB_4

#define SCB_SCR_ENABLE_SLEEPONEXIT (0x00000002)
#define ADC_TEMP_STEP_SIZE_14 (0.000201416)
#define ADC_TEMP_STEP_SIZE_12 (0.000805664)
#define ADC_TEMP_STEP_SIZE_10 (0.003222656)
#define ADC_TEMP_STEP_SIZE_8  (0.012890625)
#define CAP_COMP_VAL_100HZ 30286
#define JOY_X_CENTER 7937
#define JOY_Y_CENTER 8181
#define JITTER 2
#define X_POS 1
#define Y_POS 2

/* Function prototypes */
inline float vToC(uint32_t vSense);
inline float vToF(uint32_t vSense);
inline float vToK(uint32_t vSense);
void configureADC(void);
void configureButtons(void);
void configurePorts(void);
void configureTimerInterrupts(void);
void sendTempUART(uint32_t tempV);

/* Global vars */
static uint8_t dump = 0;     // Boolean to communicate when the buffer fills to main
static uint8_t posRead = 0;  // Boolean to communicate when joystick position is read to main
static CircBuf_T buffer;     // The buffer used in main and the ADC interrupt
static uint32_t tics = 0;    // Holds number of times timer has fired for temp
static uint32_t ticsJoy = 0; // Holds number of times timer has fired for joystick
static struct {
	uint32_t x;
	uint32_t y;
} joyPos = {0};
static char * quadrants[] = {
		"Upper Right",
		"Upper Left",
		"Lower Left",
		"Lower Right"
};

void main(void) {
	uint8_t quadrant = 0;                    // Holds the quadrant that the joy stick is in
	char str[11];                            // String buffer for number printing
	static uint32_t i = 0;
	uint32_t tempV = 0;                      // For ADC14 output
	initializeBuffer(&buffer, 60);
	configureButtons();
	configurePorts();
	configureTimerInterrupts();
	enableUART(CS_CTL0_DCORSEL_2, 115200);
    WDTCTL = WDTPW | WDTHOLD;                // Stop watchdog timer
    configureADC();                          // configure the ADC for interrupts...
    __enable_interrupt();
    SCB->SCR &= ~SCB_SCR_ENABLE_SLEEPONEXIT; // Wake up on exit from ISR
    while(1) {

    	if ((posRead & X_POS) && (posRead & Y_POS)) { // The joystick position was scanned
    		quadrant = 0;
    		if (joyPos.x > JOY_X_CENTER + JITTER) {   // Ensure we are not at the origin or axes
    			if (joyPos.y > JOY_Y_CENTER + JITTER) quadrant = 1;
    			else if (joyPos.y < JOY_Y_CENTER - JITTER) quadrant = 4;
    		}
    		else if (joyPos.x < JOY_X_CENTER - JITTER) {
    			if (joyPos.y > JOY_Y_CENTER + JITTER) quadrant = 2;
    			else if (joyPos.y < JOY_Y_CENTER - JITTER) quadrant = 3;
    		}
    		if (quadrant) { // Only print if we are not on the edge of two quadrants
            	sendStringUART(quadrants[quadrant - 1]);
            	sendNewlineUART();
    		}
        	posRead = 0;
    	}

    	if (dump) { // We received a signal to dump the buffer contents from an interrupt
    		TA0CCTL0 &= ~TIMER_A_CCTLN_CCIE; // Stop taking temperature data
    		for (i = 1; removeItem(&buffer, &tempV) != BE_EMPTY; i++) {
    			itoa(i, str);
    			sendStringUART(str);
    			sendStringUART(")  ");
    			sendNewlineUART();
    			sendTempUART(tempV);
    		}
    		dump = 0;                        // Reset signal
    		TA0CCTL0 |= TIMER_A_CCTLN_CCIE;  // Start Taking temperature data
    	}
#ifdef PB_1
    	ADC14->CTL0 |= ADC14_CTL0_SC;        // Sampling and conversion start
    	__sleep();                           // Blocks here until Conversion finishes
    	tempV = ADC14->MEM[0];
    	sendTempUART(tempV);
#endif

    }
}

// 1.9 mV / deg C
// -40 = 0
// VSense (V from ADC) = TCsensor * Temp (Want) + Vsensor (730 mV)
// Temp = (VSense - Vsensor)/TCsensor
inline float vToC(uint32_t vSense) {
	return (((float) vSense) * ADC_TEMP_STEP_SIZE_14 - .73 ) / 0.0019;
}

inline float vToF(uint32_t vSense) {
	return vToC(vSense) * 1.8 + 32;
}

inline float vToK(uint32_t vSense) {
	return vToC(vSense) + 273.15;
}

void configureADC() {
	P6SEL0 |= BIT0; // Select A16 mode
	P6SEL1 |= BIT0;
	P4SEL0 |= BIT4; // Select A9 mode
	P4SEL1 |= BIT4;
	// Initialize the shared reference module
	// By default, REFMSTR=1 => REFCTL is used to configure the internal reference
	while (REF_A->CTL0 & REF_A_CTL0_GENBUSY);               // If ref generator busy, WAIT

	REF_A->CTL0 = REF_A_CTL0_VSEL_0 |                       // Vref = 1.2
			      REF_A_CTL0_ON;                            // Enables references
	REF_A->CTL0 &= ~REF_A_CTL0_TCOFF;                       // Turn onTemperature Sensor
	ADC14->CTL0 = ADC14_CTL0_SHT0_5 |                       // Sample/hold set to 96 cycles
			      ADC14_CTL0_ON |                           // Turn on ADC
				  ADC14_CTL0_CONSEQ_1 |
				  ADC14_CTL0_SHP;                           // Pulse sample mode
	ADC14->CTL0 &= ~ADC14_CTL0_ENC;                         // Allow changes
	ADC14->CTL1 = ADC14_CTL1_TCMAP | ADC14_CTL1_RES__14BIT; // Conf internal temp sensor channel,set resolution
	ADC14->MCTL[0] = ADC14_MCTLN_INCH_22;                   // Map Temp Analog channel to MEM0/MCTL0, set 3.3v ref
	ADC14->MCTL[1] = ADC14_MCTLN_INCH_15;                   // Map joyPos.x to MEM[1]
	ADC14->MCTL[2] = ADC14_MCTLN_INCH_9 |                   // Map joyPos.y to MEM[2]
			         ADC14_MCTLN_EOS;                       // Stop ADC from checking chanels once it reaches MEM[2]
	ADC14->IER0 = ADC14_IER0_IE0 |                          // Enable MEM 0/1/2 Interrupts
			      ADC14_IER0_IE1 |
				  ADC14_IER0_IE2;
	while(!(REF_A->CTL0 & REF_A_CTL0_GENRDY));              // Wait for refgenerator to settle
	ADC14->CTL0 |= ADC14_CTL0_ENC;                          // Enable Conversions
	NVIC_EnableIRQ(ADC14_IRQn);                             // Enable ADC int in NVIC module}
}

void configureButtons(void) {
	P1DIR &= ~(BIT1 | BIT4);    // Buttons S1/S2 set to input
	P1REN |= BIT1 | BIT4;       // Enable pullup/down resistors
	P1OUT |= BIT1 | BIT4;       // Set to pullup mode
	P1IFG = 0;                  // Clear the interrupt Flag
	P1IES |= BIT1 | BIT4;       // Interrupt fires on high to low transition
	P1IE |= BIT1 | BIT4;        // Enable interrupt for buttons
	NVIC_EnableIRQ(PORT1_IRQn); // Register port 1 interrupts with NVIC
}

void configurePorts(void) {
	P1DIR |= BIT0;  // Enable LEDs
	P2DIR |= BIT2;
	P1OUT &= ~BIT0;
	P2OUT &= ~BIT2;
}

void configureTimerInterrupts(void) {
	TA0CCR0 = CAP_COMP_VAL_100HZ;         // Capture compare value
	TA0CCTL0 = TIMER_A_CCTLN_CM__RISING | // Capture on rising edge
			   TIMER_A_CCTLN_CCIE |       // TACCR0 interrupt enabled
			   TIMER_A_CCTLN_OUTMOD_1;    // Set the output bit
	TA0CTL = TIMER_A_CTL_SSEL__SMCLK |    // Use SMCLK
			 TIMER_A_CTL_ID__2 |          // Any input divider
			 TIMER_A_CTL_MC__UP;          // Count up to TA0CCR0
	TA0R = 0;                             // Start at zero
	NVIC_EnableIRQ(TA0_0_IRQn);           // Enable interrupt in NVIC
}

void sendTempUART(uint32_t tempV) {
	char str[11];
	sendStringUART("Temperature readings: ");
	sendNewlineUART();
	sendStringUART("Value: ");
	itoa(tempV, str);
	sendStringUART(str);
	sendNewlineUART();
	sendStringUART("Temp (C): ");
	sprintf(str, "%.10g", vToC(tempV));
	sendStringUART(str);
	sendNewlineUART();
	sendStringUART("Temp (F): ");
	sprintf(str, "%.10g", vToF(tempV));
	sendStringUART(str);
	sendNewlineUART();
	sendStringUART("Temp (K): ");
	sprintf(str, "%.10g", vToK(tempV));
	sendStringUART(str);
	sendNewlineUART();
	sendNewlineUART();
}

void ADC14Handler(void){
	if (ADC14->IFGR0 & ADC14_IFGR0_IFG0) { // Temp sensor
		if (addItem(&buffer, ADC14->MEM[0]) == BE_FULL) P2OUT |= BIT2;
	}
	if (ADC14->IFGR0 & ADC14_IFGR0_IFG1) { // X pos
		posRead |= X_POS;
		joyPos.x = ADC14->MEM[1];
	}
	if (ADC14->IFGR0 & ADC14_IFGR0_IFG2) { // Y pos
		posRead |= Y_POS;
		joyPos.y = ADC14->MEM[2];
	}
}

void Port1Handler(void) {
	if (P1IFG & (BIT1 | BIT4)) { // Button press
		dump = 1;                // Tell main to dump buffer
		P2OUT &= ~BIT2;          // Turn off "Buffer full" LED
	}
	P1IFG = 0;
}

void TimerA0Handler(void) {
	TA0CCTL0 &= ~CCIFG;               // Clear the Capture Compare Interrupt Flag
#ifdef PB_4
	tics++;
	if (tics == 100) {                // 1 Hz for temp sensor
		ADC14->CTL0 |= ADC14_CTL0_SC; // Sampling and conversion start
		tics = 0;
	}
#else
	ticsJoy++;
	if (ticsJoy == 3) {               // 10 Hz for joy stick (100 / (3 tics * 3 Chanels for the ADC to read))
		ADC14->CTL0 |= ADC14_CTL0_SC; // Sampling and conversion start
		ticsJoy = 0;
	}
#endif
}
