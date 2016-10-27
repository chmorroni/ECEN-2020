#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include "uart.h"
//#include "errors.h"
#include "core_cm4.h"

#define SCB_SCR_ENABLE_SLEEPONEXIT (0x00000002)
//#define ADC_TEMPERATURE_PER_VOLTAGE_STEP_SIZE (0.000201416)
#define ADC_TEMPERATURE_PER_VOLTAGE_STEP_SIZE (0.000816)

inline float vToC(uint32_t vSense);
inline float vToF(uint32_t vSense);
inline float vToK(uint32_t vSense);
void configure_ADC(void);

void test(void) {
	uint8_t data;
	if (UCA0IFG & UCRXIFG) {            // Triggered due to a receive.
		data = UCA0RXBUF;
		sendByteUART(data);
		UCA0IFG &= ~UCRXIFG;
	}
}

void main(void) {
	enableUART(CS_CTL0_DCORSEL_2, 115200);
	sendByteUART('d');
	enableUARTInterrupts(&test, EUSCI_A_IE_RXIE);
	while(1) sendByteUART('d');
//	P1DIR |= BIT0;
//	uint32_t tempV = 0;
//    WDTCTL = WDTPW | WDTHOLD;                  // Stop watchdog timer
//    /* Other micro controller configuration code here... */
//    configure_ADC();                           // configure the ADC for interrupts...
//    __enable_interrupt();
//    SCB->SCR &= ~SCB_SCR_ENABLE_SLEEPONEXIT;   // Wake up on exit from ISR
//    while(1) {
//    	P1OUT ^= BIT0;
//    	ADC14->CTL0 |= ADC14_CTL0_SC;          // Sampling and conversion start
////    	__sleep();                             // Blocks here until Conversion finishes
//    	tempV = ADC14->MEM[0];
//    	printf("Temperature readings: %10d     %10g     %10g     %10g\n", tempV, vToC(tempV), vToF(tempV), vToK(tempV));
//    }
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

void configure_ADC() {
	// Initialize the shared reference module
	// By default, REFMSTR=1 => REFCTL is used to configure the internal reference
	while (REF_A->CTL0 & REF_A_CTL0_GENBUSY);   // If ref generator busy, WAIT

	REF_A->CTL0= REF_A_CTL0_VSEL_0 | REF_A_CTL0_ON; // Enable internal 1.2V reference
	REF_A->CTL0&= ~REF_A_CTL0_TCOFF;           // Turn onTemperature Sensor
	// Configure ADC -Pulse sample mode; ADC14SC trigger
	// ADC ON,temperature sample period>30us
	ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_ON | ADC14_CTL0_SHP; // TODO: Maybe =
	ADC14->CTL0 &= ~ADC14_CTL0_ENC;                         // Allow changes
	ADC14->CTL1 = ADC14_CTL1_TCMAP | ADC14_CTL1_RES__12BIT; // Conf internal temp sensor channel,set resolution
	ADC14->MCTL[0] = ADC14_MCTLN_INCH_22;                     // Map Temp Analog channel to MEM0/MCTL0, set 3.3v ref
	ADC14->IER0 = ADC14_IER0_IE22;                          // Enable MCTL0/MEM0 Interrupts
	while(!(REF_A->CTL0 & REF_A_CTL0_GENRDY));              // Wait for refgenerator to settle
	ADC14->CTL0 |= ADC14_CTL0_ENC;                          // Enable Conversions
	NVIC_EnableIRQ(ADC14_IRQn);                             // Enable ADC int in NVIC module}
}

