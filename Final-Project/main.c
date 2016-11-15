#include <stdint.h>
#include "msp.h"
#include "core_cm4.h"
#include "Libs/nrf.h"
#define BUFF_TYPE nrfCmd
#include "Libs/buffer.h"
#include "Libs/serial.h"
#include "Libs/error.h"
#include "Libs/helpers.h"
#include "Libs/timers.h"
#include "Libs/logging.h"

void main(void) {
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;                      // Stop watchdog timer

}

#ifdef TESTING
#define PULSE_WIDTH (100)
#define DUTY_CYCLE (0.01)
#define RTC_KEY ((uint16_t) 0xA500)

Buff spiTXBuff = {0};
Buff spiRXBuff = {0};
Buff slaveTXBuff = {0};
Buff slaveRXBuff = {0};
Buff uartTXBuff = {0};
Buff uartRXBuff = {0};
uint8_t send = 0;
uint16_t time = 0;

void inline sendMaster(uint8_t byte);
void inline sendSlave(uint8_t byte);
void configureButtons(void);
void sendCommandSPI(uint8_t command);
void inline printByte(uint8_t byte);
void printBytes(uint8_t * array, uint32_t length);
void log(char * string);
void printNewline(void);


void main(void) {
	uint8_t resp = 0;
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;                      // Stop watchdog timer

	initBuff(&spiTXBuff, 6);
	initBuff(&spiRXBuff, 6);
	initBuff(&slaveTXBuff, 6);
	initBuff(&slaveRXBuff, 6);
	initBuff(&uartTXBuff, 32);
	initBuff(&uartRXBuff, 6);

	/* Setup RTC
	 */
	RTC_C->CTL0 = RTC_KEY;               // Unlock register
	RTC_C->CTL0 = RTC_C_CTL0_RDYIE |     // Interrupt every second to update time
	              RTC_KEY;               // Stay unlocked
	RTC_C->CTL13 = RTC_C_CTL13_RDY;      // Calendar mode (0 is reserved)
	RTC_C->CTL0 &= ~RTC_C_CTL0_KEY_MASK; // Relock by clearing key
//	NVIC_EnableIRQ(RTC_C_IRQn);

	/* Setup UART
	 */
	enableUART(CS_CTL0_DCORSEL_1, 115200);
	enableInterruptsUART(NULL, EUSCI_A_IE_RXIE);
	log("           Test!!!!");
	printNewline();

	/* For SPI
	 */

	/* Setup led
	 */
	P1->SEL1 &= ~BIT0;
	P1->SEL0 &= ~BIT0;
	P1->DIR |= BIT0;
	P1->OUT &= ~BIT0;

	TIMER_A1->CCR[0] = 0;                          // Stop timer for config
	TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK |      // Use SMCLK
	                TIMER_A_CTL_ID__8 |            // Input divider
	                TIMER_A_CTL_MC__UP;            // Count up to TA1CCR0
	TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE;
	TIMER_A1->CTL |= TIMER_A_CTL_CLR;              // Reset the timer
	TIMER_A1->CCR[0] = 60000;
	NVIC_EnableIRQ(TA1_0_IRQn);

	/* Setup input buttons
	 */
	configureButtons();

	/* Setup eUSCI_A1 for SPI
	 */
	EUSCI_A1_SPI->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI_A1 in reset
	EUSCI_A1_SPI->CTLW0 = EUSCI_A_CTLW0_CKPH |   // phase = 1 to shift on falling edge
	                      EUSCI_A_CTLW0_MSB |    // MSB first
	                      EUSCI_A_CTLW0_MST |    // Master
	                      EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                      EUSCI_A_CTLW0_SYNC |   // SPI mode
	                      EUSCI_A_CTLW0_SSEL__SMCLK |
	                      EUSCI_A_CTLW0_STEM |   // STE used for slave
	                      EUSCI_A_CTLW0_SWRST;   // STE used for slave
	EUSCI_A1_SPI->BRW = 8;                       // Divide by enough to safely use 48MHz
	EUSCI_A1_SPI->STATW = 0;                     // Defaults

	/* Setup pin for SPI
	 */
	P2->SEL1 &= ~(BIT3 | BIT5 | BIT6 | BIT7);
	P2->SEL0 |= BIT3 | BIT5 | BIT6 | BIT7;
	PMAP->KEYID = PMAP_KEYID_VAL;                // Unlock PMAP controller
	PMAP->CTL |= PMAP_CTL_PRECFG;                // Allow it to be reconfigured later
	P2MAP->PMAP_REGISTER3 = PMAP_UCA1STE;
	P2MAP->PMAP_REGISTER5 = PMAP_UCA1CLK;
	P2MAP->PMAP_REGISTER6 = PMAP_UCA1SOMI;
	P2MAP->PMAP_REGISTER7 = PMAP_UCA1SIMO;
	PMAP->KEYID = 0;                             // Lock PMAP controller
	EUSCI_A1_SPI->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Release for operation
	EUSCI_A1_SPI->IE = EUSCI_A_IE_TXIE |         // TX interrupt
	                   EUSCI_A_IE_RXIE;          // RX interrupt
	NVIC_EnableIRQ(EUSCIA1_IRQn);

	/* Setup extra pins to NRF
	 */
	P3->DIR |= BIT5 | BIT7;
	P3->OUT &= ~(BIT7);
	P3->OUT |= BIT5;

	P6->DIR &= ~BIT6;
	P6->REN |= BIT6;            // Enable pullup/down resistors
	P6->OUT |= BIT6;            // Set to pullup mode
	P6->IFG = 0;                // Clear the interrupt Flag
	P6->IES |= BIT6;            // Interrupt fires on high to low transition
	P6->IE |= BIT6;             // Enable interrupt for buttons
	NVIC_EnableIRQ(PORT6_IRQn); // Register port 1 interrupts with NVIC










	/* Setup eUSCI_A1 for SPI (test slave)
	 */
	EUSCI_A2_SPI->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI_A2 in reset
	EUSCI_A2_SPI->CTLW0 = EUSCI_A_CTLW0_CKPH |
	                      EUSCI_A_CTLW0_MSB |    // MSB first
	                      EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                      EUSCI_A_CTLW0_SYNC |   // SPI mode
	                      EUSCI_A_CTLW0_SWRST;   // STE used for slave
	EUSCI_A2_SPI->STATW = 0;                     // Defaults

	/* Setup pin for SPI
	 */
	P3->SEL1 &= ~(BIT3 | BIT5 | BIT6 | BIT7);
	P3->SEL0 |= BIT3 | BIT5 | BIT6 | BIT7;
	PMAP->KEYID = PMAP_KEYID_VAL;                // Unlock PMAP controller
	PMAP->CTL |= PMAP_CTL_PRECFG;                // Allow it to be reconfigured later
	P3MAP->PMAP_REGISTER3 = PMAP_UCA2STE;
	P3MAP->PMAP_REGISTER5 = PMAP_UCA2CLK;
	P3MAP->PMAP_REGISTER6 = PMAP_UCA2SOMI;
	P3MAP->PMAP_REGISTER7 = PMAP_UCA2SIMO;
	PMAP->KEYID = 0;                             // Lock PMAP controller
	EUSCI_A2_SPI->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Release for operation
	EUSCI_A2_SPI->IE = EUSCI_A_IE_TXIE |        // TX interrupt
	                   EUSCI_A_IE_RXIE;         // RX interrupt
	EUSCI_A2_SPI->IFG = 0;
	NVIC_EnableIRQ(EUSCIA2_IRQn);
	EUSCI_A1_SPI->TXBUF = 0xBEEF;











	/* Setup the PWM output on P2.6 for the LCD. The ordering is important here,
	 * we stop the timer first, then setup the general control register,
	 * followed by CCTL0 for the pulse rate, then the CCTLn registers to set the
	 * duty cycle. After this we reset the timer and start it counting again by
	 * setting CCR0 to a non-zero value.
	 */
	TIMER_A0->CCR[0] = 0;                          // Stop timer for config
	TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |      // Use SMCLK
	                TIMER_A_CTL_ID__1 |            // Input divider
	                TIMER_A_CTL_MC__UP;            // Count up to TA0CCR0
	TIMER_A0->CCTL[0] = 0;                         // Defaults look good
	TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;    // Set/Reset
	TIMER_A0->CCR[1] = (uint16_t) (DUTY_CYCLE * PULSE_WIDTH);

	/* Setup pin for PWM
	 * Needs to attach to OUT3 of TA0 (See page 147 of specific DS)
	 */
	P2->DIR |= BIT2;
	P2->SEL1 &= ~BIT2;
	P2->SEL0 |= BIT2;
	PMAP->KEYID = PMAP_KEYID_VAL;                  // Unlock PMAP controller
	PMAP->CTL |= PMAP_CTL_PRECFG;                  // Allow it to be reconfigured later
	P2MAP->PMAP_REGISTER2 = PMAP_TA0CCR1A;         // Tie to OUT1 from TA0
	PMAP->KEYID = 0;                               // Lock PMAP controller

	TIMER_A0->CTL |= TIMER_A_CTL_CLR;              // Reset the timer
	TIMER_A0->CCR[0] = PULSE_WIDTH;                // Start timer at 200Hz
	__enable_interrupts();

	while(1) {
		time++;
		if (send) {
			sendMaster('F');
			send = 0;
		}
		if (getFromBuff(&uartRXBuff, &resp) == BUFF_NO_ERR) {
			log("UART got: ");
			printByte(resp);
			printNewline();
		}
		if (getFromBuff(&spiRXBuff, &resp) == BUFF_NO_ERR) {
			log("Master got: ");
			printByte(resp);
			printNewline();
		}
		if (getFromBuff(&slaveRXBuff, &resp) == BUFF_NO_ERR) {
			log("Slave got: ");
			printByte(resp);
			printNewline();
			sendSlave(resp);
		}
	}
}





void sendCommandSPI(uint8_t command) {
	if (!(EUSCI_A1_SPI->IFG & EUSCI_A_IFG_TXIFG)) {
		while (addToBuff(&spiTXBuff, command) != BUFF_NO_ERR); // Keep trying until it works
	}
	else EUSCI_A1_SPI->TXBUF = command;
}

void inline sendMaster(uint8_t byte) {
	log("Master sending: ");
	printByte(byte);
	printNewline();
	addToBuff(&spiTXBuff, byte);
	addToBuff(&spiTXBuff, byte - 3);
	EUSCI_A1_SPI->IE |= EUSCI_A_IE_TXIE;
}

void inline sendSlave(uint8_t byte) {
	log("Slave sending: ");
	printByte(byte);
	printNewline();
	addToBuff(&slaveTXBuff, byte);
	EUSCI_A2_SPI->IE |= EUSCI_A_IE_TXIE;
}






void inline printByte(uint8_t byte) {
	while(addToBuff(&uartTXBuff, byte) != BUFF_NO_ERR);
	EUSCI_A0->IE |= EUSCI_A_IE_TXIE;
}

void printBytes(uint8_t * array, uint32_t length) {
	while (length--) printByte(*(array++)); // Increment up array, printing byte by byte until length is hit
}

void log(char * string) {
	char nowStr[12];
	char * nowPtr = nowStr;
//	itoa((uint32_t) RTC_C->TIM0, nowStr);
	itoa((uint32_t) time, nowStr);
		while (*nowPtr) printByte(*(nowPtr++));
	printByte(':');
	printByte(' ');
	while (*string) printByte(*(string++)); // Increment up string, printing char by char until '\0' is hit
}

void printNewline(void) {
	printByte('\n'); // Transmit a line break
	printByte('\r');
}






/* Interrupt handlers
 */

void TA1Handler(void) {
	P1OUT ^= BIT0;
	TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
	send = 1;
}

void Port1Handler(void) {
	P1OUT ^= BIT0;
	if (P1->IFG & BIT1) {
		log("Hey!");
		printNewline();
	}
	if (P1->IFG & BIT4) {
		log("Hey!");
		printNewline();
	}
	P1->IFG = 0;
}

void Port6Handler(void) {
	// NRF received somthin'
	P6->IFG = 0;
}

void UARTHandler(void) {
	uint8_t data;
	if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) {
		addToBuff(&uartRXBuff, EUSCI_A0->RXBUF);
	}
	if (EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&uartTXBuff, &data) == BUFF_NO_ERR) EUSCI_A0->TXBUF = data;
		else EUSCI_A0->IE &= ~EUSCI_A_IE_TXIE;
	}
}

void SPIHandler(void) {
	nrfCmd command = 0;
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_RXIFG) {
		addToBuff(&spiRXBuff, EUSCI_A1_SPI->RXBUF);
	}
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&spiTXBuff, &command) == BUFF_NO_ERR) EUSCI_A1_SPI->TXBUF = command;
		else EUSCI_A1_SPI->IE &= ~EUSCI_A_IE_TXIE;
	}
}

void SPISlaveHandler(void) {
	uint8_t data = 0;
	if (EUSCI_A2_SPI->IFG & EUSCI_A_IFG_RXIFG) {
		addToBuff(&slaveRXBuff, EUSCI_A2_SPI->RXBUF - 1);
	}
	if (EUSCI_A2_SPI->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&slaveTXBuff, &data) == BUFF_NO_ERR) EUSCI_A2_SPI->TXBUF = data;
		else EUSCI_A2_SPI->IE &= ~EUSCI_A_IE_TXIE;
	}
}

void RTCHandler(void) {
	P1OUT ^= BIT0;
//	RTC_C->CTL0 = RTC_KEY;               // Unlock register
//	RTC_C->CTL0 |= RTC_C_CTL0_RDYIE;     // Interrupt every second to update time
	time = RTC_C->TIM0;
//	RTC_C->CTL0 &= ~RTC_C_CTL0_RDYIFG;
//	RTC_C->CTL0 &= ~RTC_C_CTL0_KEY_MASK; // Relock by clearing key
}










void configureButtons(void) {
	P1->DIR &= ~(BIT1 | BIT4);  // Buttons S1/S2 set to input
	P1->REN |= BIT1 | BIT4;     // Enable pullup/down resistors
	P1->OUT |= BIT1 | BIT4;     // Set to pullup mode
	P1->IFG = 0;                // Clear the interrupt Flag
	P1->IES |= BIT1 | BIT4;     // Interrupt fires on high to low transition
	P1->IE |= BIT1 | BIT4;      // Enable interrupt for buttons
	NVIC_EnableIRQ(PORT1_IRQn); // Register port 1 interrupts with NVIC
}
#endif
