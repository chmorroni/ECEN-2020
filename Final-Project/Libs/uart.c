#include "uart.h"

static void nop(void) {}
static interruptFuncPtr callback = &nop;

/**
 * The function that is called by the interrupt. A function
 * pointer variable cannot be used as the interrupt needs a
 * const pointer. PUT THIS IN THE ARRAY IN THE STARTUP FILE!
 */
void eUSCIA0Handler(void) {
	(*callback)();
}

/**
 * freq is CS_CTL0_DCORSEL_0 (1.5 MHz) to CS_CTL0_DCORSEL_5 (48 MHz)
 *   CS_CTL0_DCORSEL_0 // Nominal DCO Frequency Range (MHz): 1 to 2
 *   CS_CTL0_DCORSEL_1 // Nominal DCO Frequency Range (MHz): 2 to 4
 *   CS_CTL0_DCORSEL_2 // Nominal DCO Frequency Range (MHz): 4 to 8
 *   CS_CTL0_DCORSEL_3 // Nominal DCO Frequency Range (MHz): 8 to 16
 *   CS_CTL0_DCORSEL_4 // Nominal DCO Frequency Range (MHz): 16 to 32
 *   CS_CTL0_DCORSEL_5 // Nominal DCO Frequency Range (MHz): 32 to 64
 */
void enableUART(uint32_t freq, uint32_t baud) {
	float mhz = 1.5 * (1 << (freq >> 16));          // bit pattern to MHz
	float n = mhz * 1000000 / baud;                 // N = freqBRCLK / baud
	float n16 = n / 16;
	uint32_t intn = (uint32_t) (n > 16 ? n16 : n);  // INT(N/16)
	float fracn = ((n > 16 ? n16 : n) - intn);      // N - INT(N/16)

	/* Table 22-4 in family datasheet */
	uint32_t brs = fracn >= 0.9288 ? 0xFE :
	               fracn >= 0.9170 ? 0xFD :
	               fracn >= 0.9004 ? 0xFB :
	               fracn >= 0.8751 ? 0xF7 :
	               fracn >= 0.8572 ? 0xEF :
	               fracn >= 0.8464 ? 0xDF :
	               fracn >= 0.8333 ? 0xBF :
	               fracn >= 0.8004 ? 0xEE :
	               fracn >= 0.7861 ? 0xED :
	               fracn >= 0.7503 ? 0xDD :
	               fracn >= 0.7147 ? 0xBB :
	               fracn >= 0.7001 ? 0xB7 :
	               fracn >= 0.6667 ? 0xD6 :
	               fracn >= 0.6432 ? 0xB6 :
	               fracn >= 0.6254 ? 0xB5 :
	               fracn >= 0.6003 ? 0xAD :
	               fracn >= 0.5715 ? 0x6B :
	               fracn >= 0.5002 ? 0xAA :
	               fracn >= 0.4378 ? 0x55 :
	               fracn >= 0.4286 ? 0x53 :
	               fracn >= 0.4003 ? 0x92 :
	               fracn >= 0.3753 ? 0x52 :
	               fracn >= 0.3575 ? 0x4A :
	               fracn >= 0.3335 ? 0x49 :
	               fracn >= 0.3000 ? 0x25 :
	               fracn >= 0.2503 ? 0x44 :
	               fracn >= 0.2224 ? 0x22 :
	               fracn >= 0.2147 ? 0x21 :
	               fracn >= 0.1670 ? 0x11 :
	               fracn >= 0.1430 ? 0x20 :
	               fracn >= 0.1252 ? 0x10 :
	               fracn >= 0.1001 ? 0x08 :
	               fracn >= 0.0835 ? 0x04 :
	               fracn >= 0.0715 ? 0x02 :
	               fracn >= 0.0529 ? 0x01 : 0;

	/* Configure required clocks */
	CS->KEY = 0x695A;           // Unlock CS module for register access
	CS->CTL0 = freq;            // Setup DCO Clock to Requested frequency
	CS->CTL1 = CS_CTL1_SELA_2 | // ACLK source set to REFOCLK
	           CS_CTL1_SELS_3 | // SMCLK set to DCOCLK
	           CS_CTL1_SELM_3;  // MCLK set to DCOCLK
	CS->KEY = 0;                // lock CS module for register access

	/* Configure serial port */
	P1SEL0 |= BIT2 | BIT3;                  // 2 = RX 3 = TX
	P1SEL1 &= ~(BIT2 | BIT3);               // Set to primary function (01)
	UCA0CTLW0 |= UCSWRST;                   // Put eUSCI in reset
	UCA0CTLW0 = EUSCI_A_CTLW0_SSEL__SMCLK | // Use SMCLK
	            UCSWRST;                    // Keep eUSCI in reset

	if (n > 16) {                                                        // Need to use oversampling mode
		UCA0BRW = intn;                                                  // Set clock scaler to INT(N/16)
		UCA0MCTLW = ((uint32_t) (fracn * 16)) << EUSCI_A_MCTLW_BRF_OFS | // INT([(N/16) – INT(N/16)] × 16
		            brs << EUSCI_A_MCTLW_BRS_OFS |                       // Modulation pattern (see table above)
		            EUSCI_A_MCTLW_OS16;                                  // Use modulator
	}
	else {                                        // Don't need to use oversampling mode
		UCA0BRW = intn;                           // Set clock scaler to N
		UCA0MCTLW = brs << EUSCI_A_MCTLW_BRS_OFS; // Modulation pattern (see table above)
	}

	UCA0CTLW0 &= ~UCSWRST; // Initialize eUSCI
}

/**
 * @param func - The function to call for the interrupt
 * @param interrupts - A bitmask of any combination of:
 *   EUSCI_A_IE_RXIE    // Receive interrupt enable
 *   EUSCI_A_IE_TXIE    // Transmit interrupt enable
 *   EUSCI_A_IE_STTIE   // Start bit interrupt enable
 *   EUSCI_A_IE_TXCPTIE // Transmit complete interrupt enable
 */
void enableUARTInterrupts(interruptFuncPtr func, uint16_t interrupts) {
	callback = func;
	UCA0IE &= ~EUSCI_A_RXBUF_RXBUF_MASK;
	UCA0IE |= interrupts;                       // Enable USCI_A0 RX interrupt
	NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31); // Enable eUSCIA0 interrupt in NVIC
}

void inline sendByteUART(uint8_t tx_data) {
    P1OUT |= BIT0;                // Turn on LED1 to indicate ‘waiting to transmit’
    while (!(UCA0IFG & UCTXIFG)); // Block until transmitter is ready
    UCA0TXBUF = tx_data;          // Load data onto buffer
    P1OUT &= ~BIT0;               // Turn off LED1 to indicate ‘transmitting has started’
}

void sendBytesUART(uint8_t * array, uint32_t length) {
	while (length--) sendByteUART(*(array++)); // Increment up array, printing byte by byte until length is hit
}

void sendStringUART(char * string) {
	while (*string) sendByteUART(*(string++)); // Increment up string, printing char by char until '\0' is hit
}

void sendNewlineUART(void) {
	sendByteUART('\n'); // Transmit a line break
	sendByteUART('\r');
}
