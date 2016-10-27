#include "msp.h"

void main(void)
{
	
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
	
    UCA0CTLW0 |= UCSWRST;       // Put eUSCI in reset

    // All your configuration code goes here

    UCA0CTLW0 &= ~UCSWRST;      // Initialize eUSCI
}

void configure_clocks(void) {

	CS->KEY = 0x695A;		// Unlock CS module for register access
//	CS->CTL0 = 0;			// Reset tuning parameters
	CS->CTL0 = 0x00040000;			// Setup DCO Clock

	// Select ACLK = REFO, SMCLK = MCLK = DCO
	CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
	CS->KEY = 0;				// lock CS module for register access

}

void configure_serial_port() {

	// ConfigureUART pins, set 2-UART pin as primary function
	P1SEL0 |= BIT1 | BIT2;
	P1SEL1 &= ~(BIT1 | BIT2);

	// Configure UART
	UCA0CTLW0 |= UCSWRST;		// Put eUSCI in reset
	UCA0CTLW0 = 0x0081;			// Select Frame parameters and clock source
	UCA0BR0 = 0x71;              // Set Baud Rate to 625
	UCA0BR1 = 0x02;
//	UCA0MCTLW = ;            // Set first stage modulator bits(if necessary)
	UCA0CTLW0 &= ~UCSWRST;      // Initialize eUSCI
	UCA0IE |= 0x03;              // Enable USCI_A0 RX interrupts

	NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);	// Enable eUSCIA0 interrupt in NVIC

}

void uart_putchar(uint8_t tx_data) {
	// TODO
	while(0);				// Block until transmitter is ready
	UCA0TXBUF = tx_data;		// Load data onto buffer

}

void uart_putchar_n(uint8_t* data, uint32_t length) {
	uint32_t i = -1;
	while (++i < length) uart_putchar(data[i]);
}

