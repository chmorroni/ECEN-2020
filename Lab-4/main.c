#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include "Libs/uart.h"
#include "Libs/buffer_fifo.h"
//#include "errors.h"
#include "core_cm4.h"

#define PB_1

#define SCB_SCR_ENABLE_SLEEPONEXIT (0x00000002)
#define ADC_TEMPERATURE_PER_VOLTAGE_STEP_SIZE (0.012890625)
#define CAP_COMP_VAL_100HZ 30286

inline float vToC(uint32_t vSense);
inline float vToF(uint32_t vSense);
inline float vToK(uint32_t vSense);
void configureADC(void);
void configureButtons(void);
void configurePorts(void);
void configureTimerInterrupts(void);
void sendTempUART(uint32_t tempV);

static uint8_t dump = 0;
static CircBuf_T buffer;
static uint32_t tics = 0;        // Holds number of times timer has fired

void main(void) {
	char str[11];
	static uint32_t i = 0;
	initializeBuffer(&buffer, 60);
	uint32_t tempV = 0;
	configureButtons();
	configurePorts();
	configureTimerInterrupts();
	enableUART(CS_CTL0_DCORSEL_2, 115200);
	// enableUARTInterrupts(&test, EUSCI_A_IE_RXIE);
    WDTCTL = WDTPW | WDTHOLD;                  // Stop watchdog timer
    /* Other micro controller configuration code here... */
    configureADC();                           // configure the ADC for interrupts...
    __enable_interrupt();
    SCB->SCR &= ~SCB_SCR_ENABLE_SLEEPONEXIT;   // Wake up on exit from ISR
    while(1) {
    	for (i = 1; dump; i++) {
    		P2OUT &= ~BIT2;
    		if (removeItem(&buffer, &tempV) != BE_EMPTY) {
    			itoa(i, str);
    			sendStringUART(str);
    			sendStringUART(")  ");
//    			sendNewlineUART();
    			sendTempUART(tempV);
    		}
    		else dump = 0;
    	}
#ifdef PB_1
    	ADC14->CTL0 |= ADC14_CTL0_SC;          // Sampling and conversion start
    	__sleep();                             // Blocks here until Conversion finishes
    	tempV = ADC14->MEM[0];
    	sendTempUART(tempV);
#endif

    }
}

// 1.9 mV / deg C
// -40 = 0
// 0.204 mV/ADC val
// VSense (V from ADC) = TCsensor * Temp (Want) + Vsensor (730 mV (3624 steps))
// Temp = (VSense - Vsensor)/TCsensor
inline float vToC(uint32_t vSense) {
	return (((float) vSense) * ADC_TEMPERATURE_PER_VOLTAGE_STEP_SIZE - .73 ) / 0.0019;
}

inline float vToF(uint32_t vSense) {
	return vToC(vSense) * 1.8 + 32;
}

inline float vToK(uint32_t vSense) {
	return vToC(vSense) + 273.15;
}

void configureADC() {
	// Initialize the shared reference module
	// By default, REFMSTR=1 => REFCTL is used to configure the internal reference
	while (REF_A->CTL0 & REF_A_CTL0_GENBUSY);   // If ref generator busy, WAIT

	REF_A->CTL0 = REF_A_CTL0_VSEL_0 | // Vref = 1.2
			      REF_A_CTL0_ON;      // Enables references
	REF_A->CTL0 &= ~REF_A_CTL0_TCOFF; // Turn onTemperature Sensor
	// Configure ADC -Pulse sample mode; ADC14SC trigger
	// ADC ON,temperature sample period>30us
	ADC14->CTL0 = ADC14_CTL0_SHT0_5 | // Sample/hold set to 96 cycles
			      ADC14_CTL0_ON |     // Turn on ADC
				  ADC14_CTL0_SHP;     //
	ADC14->CTL0 &= ~ADC14_CTL0_ENC;                         // Allow changes
	ADC14->CTL1 = ADC14_CTL1_TCMAP | ADC14_CTL1_RES__8BIT; // Conf internal temp sensor channel,set resolution
	ADC14->MCTL[0] = ADC14_MCTLN_INCH_22;                   // Map Temp Analog channel to MEM0/MCTL0, set 3.3v ref
	ADC14->IER0 = ADC14_IER0_IE0;                           // Enable MCTL0/MEM0 Interrupts
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
	P1DIR |= BIT0;
	P2DIR |= BIT2;
	P1OUT &= ~BIT0;
	P2OUT &= ~BIT2;
}

void configureTimerInterrupts(void) {
	TA0CCR0 = CAP_COMP_VAL_100HZ;               // Capture compare value
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
	if (addItem(&buffer, ADC14->MEM[0]) == BE_FULL) P2OUT |= BIT2;
}

void Port1Handler(void) {
	if (P1IFG & (BIT1 | BIT4)){       // Button press
		dump = 1;                     // Tell main to dump buffer
	}
	P1IFG = 0;
}

void TimerA0Handler(void) {
	TA0CCTL0 &= ~CCIFG;               // Clear the Capture Compare Interrupt Flag
	tics++;
	if (tics == 100) {
		ADC14->CTL0 |= ADC14_CTL0_SC; // Sampling and conversion start
		tics = 0;
	}
}
