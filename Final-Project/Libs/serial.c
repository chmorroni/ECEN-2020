#include "serial.h"

static void nop(void) {}

static interruptFuncPtr callback = &nop;

/**
 * @desc The function that is called by the interrupt. A function
 *       pointer variable cannot be used as the interrupt needs a
 *       const pointer. PUT THIS IN THE ARRAY IN THE STARTUP FILE!
 */
void eUSCIA0Handler(void) {
	(*callback)();
}

/**
 * @desc  Sets the processor to run at the given frequency, calculates the
 *        register values for a given baud rate, and configures the UART pins
 * @param freq should be one of the below
 *   CS_CTL0_DCORSEL_0 // Nominal DCO Frequency Range (MHz): 1 to 2
 *   CS_CTL0_DCORSEL_1 // Nominal DCO Frequency Range (MHz): 2 to 4
 *   CS_CTL0_DCORSEL_2 // Nominal DCO Frequency Range (MHz): 4 to 8
 *   CS_CTL0_DCORSEL_3 // Nominal DCO Frequency Range (MHz): 8 to 16
 *   CS_CTL0_DCORSEL_4 // Nominal DCO Frequency Range (MHz): 16 to 32
 *   CS_CTL0_DCORSEL_5 // Nominal DCO Frequency Range (MHz): 32 to 64
 * @param baud is the baud rate desired. Standard rates recommended, defaults to
 *        9600 if given 0
 */
void enableUART(uint32_t freq, uint32_t baud) {
	float mhz, n, n16, fracn;
	uint32_t intn, brs;

	/* Configure required clocks */
	CS->KEY = 0x695A;           // Unlock CS module for register access
	CS->CTL0 = freq;            // Setup DCO Clock to Requested frequency
	CS->CTL1 = CS_CTL1_SELA_2 | // ACLK source set to REFOCLK
	           CS_CTL1_SELS_3 | // SMCLK set to DCOCLK
	           CS_CTL1_SELM_3;  // MCLK set to DCOCLK
	CS->KEY = 0;                // lock CS module for register access

	/* Configure serial port */
	P1->SEL0 |= BIT2 | BIT3;                      // 2 = RX 3 = TX
	P1->SEL1 &= ~(BIT2 | BIT3);                   // Set to primary function (01)
	EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;       // Put eUSCI in reset
	EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SSEL__SMCLK | // Use SMCLK
	                  EUSCI_A_CTLW0_SWRST;        // Keep eUSCI in reset

	if (!baud) baud = 9600;                       // Default to 9600
	mhz = 1.5 * (1 << (freq >> 16));              // bit pattern to MHz
	n = mhz * 1000000 / baud;                     // N = freqBRCLK / baud
	n16 = n / 16;
	intn = (uint32_t) (n > 16 ? n16 : n);         // INT(N/16)
	fracn = ((n > 16 ? n16 : n) - intn);          // N - INT(N/16)

	/* Table 22-4 in family datasheet */
	brs = fracn >= 0.9288 ? 0xFE :
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

	if (n > 16) {                                                              // Need to use oversampling mode
		EUSCI_A0->BRW = intn;                                                  // Set clock scaler to INT(N/16)
		EUSCI_A0->MCTLW = ((uint32_t) (fracn * 16)) << EUSCI_A_MCTLW_BRF_OFS | // INT([(N/16) – INT(N/16)] × 16
		                  brs << EUSCI_A_MCTLW_BRS_OFS |                       // Modulation pattern (see table above)
		                  EUSCI_A_MCTLW_OS16;                                  // Use modulator
	}
	else {                                              // Don't need to use oversampling mode
		EUSCI_A0->BRW = intn;                           // Set clock scaler to N
		EUSCI_A0->MCTLW = brs << EUSCI_A_MCTLW_BRS_OFS; // Modulation pattern (see table above)
	}

	EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
}

/**
 * @param func - The function to call for the interrupt
 * @param interrupts - A bitmask of any combination of:
 *   EUSCI_A_IE_RXIE    // Receive interrupt enable
 *   EUSCI_A_IE_TXIE    // Transmit interrupt enable
 *   EUSCI_A_IE_STTIE   // Start bit interrupt enable
 *   EUSCI_A_IE_TXCPTIE // Transmit complete interrupt enable
 */
void enableInterruptsUART(interruptFuncPtr func, uint16_t interrupts) {
	if (func) callback = func;
	EUSCI_A0->IE &= ~EUSCI_A_RXBUF_RXBUF_MASK; // ??????? RXBUFF?!?!?!?!?
	EUSCI_A0->IE |= interrupts;                       // Enable USCI_A0 RX interrupt
	NVIC_EnableIRQ(EUSCIA0_IRQn); // Enable eUSCIA0 interrupt in NVIC
}

void inline sendByteUART(uint8_t tx_data) {
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); // Block until transmitter is ready
    EUSCI_A0->TXBUF = tx_data;          // Load data onto buffer
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
