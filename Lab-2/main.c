#include "msp.h"

#undef PB2
#undef WAIT_FOR_DEBOUNCE             // Run the delay for loop for debouncing
#undef PB3
#undef PB3A
#undef PB4
#undef PB5
#undef PB9
#undef PB10
#undef PB11

#define DELAY_COUNT 20                // Cycles to wait in button interrupts to debounce switch
#define INTERRUPT_DELAY 5
#define RGB_BITS (BIT0 | BIT1 | BIT2) // Bit mask for RGB LED bits
#define CYCLING (P2OUT & RGB_BITS)    // True if any of the RGB LED is on
#define CAP_COMP_VAL 65000
#define CAP_COMP_VAL_5HZ 37765

void configure_pins(void);
void Port1Handler(void);
void configure_timer_interrupts(void);
inline void incrementRGBLED(void);
inline void decrementRGBLED(void);

int main(void) {
	volatile int i = 0;                        // Used for for loops
	WDTCTL = WDTPW | WDTHOLD;                  // Turn off watchdog timer
	configure_pins();                          // Setup inputs, output, and port interrupts
	configure_timer_interrupts();
	__enable_interrupt();                      // This enables interrupts globally for the CPU
	while (1) {
#ifdef PB3
		P1OUT &= ~BIT7;                        // Turn off BIT7
		P1IFG |= BIT6;                         // Fire interrupt, which turns it back on
		for (i = 0; i < INTERRUPT_DELAY; i++); // Wait for interrupt to fire (Takes 25.1 microseconds)
#elif defined(PB3A)
		P1OUT |= BIT7;                         // Turn on BIT7
		P1IFG |= BIT6;                         // Fire interrupt, which turns P1.7 off at end
		for (i = 0; i < INTERRUPT_DELAY; i++){ // Wait for interrupt to fire (Takes 7.93 microseconds)
			P1OUT |= BIT7;                     // turn on BIT7 as soon as main regains control
		}
#endif
	}
}

/*--------------------------------------------------------------------
* configure_pins()
*
* Configures the Port 2 buttons to be setup for input and for
* for interrupts
--------------------------------------------------------------------*/
void configure_pins(void) {
	P1DIR |= BIT0 | BIT7;           // LED1 and P1.7 set to output
	P1OUT &= ~(BIT0 | BIT7);        // Turn off LED1 and P1.7
	P2DIR |= BIT0 | BIT1 | BIT2;    // LED2 set to output
	P2OUT &= ~(BIT0 | BIT1 | BIT2); // Turn off LED2
	P1DIR &= ~(BIT1 | BIT4 | BIT6); // Buttons S1/S2 and P1.6 set to input
	P1REN |= BIT1 | BIT4;           // Enable pullup/down resistors
	P1OUT |= BIT1 | BIT4;           // Set to pullup mode
	P1IFG = 0;                      // Clear the interrupt Flag
	P1IES |= BIT1 | BIT4 | BIT6;    // Interrupt fires on high to low transition
	P1IE |= BIT1 | BIT4 | BIT6;     // Enable interrupt for buttons and P1.6
	NVIC_EnableIRQ(PORT1_IRQn);     // Register port 1 interrupts with NVIC
}

/*--------------------------------------------------------------------
* Port1Handler()
*
* The ISR for the Port1 interrupts. Register this in the Interrupt
* vector table and this will be called when the interrupt condition
* is met. An interrupt flag will be set and it needs to cleared the
* when exiting the interrupt so the request doesn’t happen after
* returning from the function. This one function will get called
* for any of the port pin interrupts for Port 1 (0-7). They share
* one interrupt vector.
--------------------------------------------------------------------*/
void Port1Handler(void) {
	volatile uint32_t i;
	static uint32_t irLEDCounter = 0;
#ifdef PB3
	P1OUT |= BIT7;                         // Turn on P1.7 right away
#endif

#ifdef PB2
	if (P1IFG & BIT1) {                    // Button 1
		P1OUT ^= BIT0;                     // Toggle LED1
	}
#elif defined(PB5)
	if (P1IFG & BIT6) {
		P1OUT ^= (BIT0 | BIT7);
	}
#endif

#ifdef PB4
	if (P1IFG & BIT1) {                    // Button 1
		incrementRGBLED();
	}
	if (P1IFG & BIT4) {                    // Button 2
		decrementRGBLED();
	}
#elif defined(PB10)
	if (P1IFG & (BIT1 | BIT4)) {            // Buttons 1 and 2
		if (!CYCLING) {                     // If RBG LED is still going, do nothing
			incrementRGBLED();
			TA0R = 0;                       // Reset the timer
			TA0CCTL0 |= TIMER_A_CCTLN_CCIE; // Turn on the timer interrupt
		}
	}
#endif

#ifdef PB11
	if (P1IFG & BIT6) {
		irLEDCounter++;
	}
#endif

#ifdef WAIT_FOR_DEBOUNCE
	for(i = 0; i < DELAY_COUNT; i++);  // Delay for switch debounce
#endif
	P1IFG = 0;                              // Clear the interrupt Flag
#ifdef PB3A
	P1OUT &= ~BIT7;                         // Turn off P1.7 right before returning
#endif
}

void configure_timer_interrupts(void) {
#if defined(PB9) || defined(PB10)
	TA0CCR0 = CAP_COMP_VAL_5HZ;           // 10 Hz with prescaler of 8, resulting in 5 Hz due to toggle
#else
	TA0CCR0 = CAP_COMP_VAL;               // Capture Compare Value
#endif
	TA0CCTL0 = TIMER_A_CCTLN_CM__RISING | // Capture on rising edge
#if defined(PB5) || defined(PB9)          // Don't default it to on if on PB10
			   TIMER_A_CCTLN_CCIE |       // TACCR0 interrupt enabled
#endif
			   TIMER_A_CCTLN_OUTMOD_1;    // Set the output bit
	TA0CTL = TIMER_A_CTL_SSEL__SMCLK |    // Use SMCLK
#if defined(PB9) || defined(PB10)
			 TIMER_A_CTL_ID__8 |          // Must be 8 for PB9 and PB10
#else
			 TIMER_A_CTL_ID__4 |          // Any input divider
#endif
			 TIMER_A_CTL_MC__UP;          // Count up to TA0CCR0
	TA0R = 0;                             // Start at zero
	NVIC_EnableIRQ(TA0_0_IRQn);           // Enable interrupt in NVIC
}

/*--------------------------------------------------------------------
* TimerA0_0IsrHandler()
*
* The ISR for the Timer interrupts. Register this in the Interrupt
* vector table and this will be called when the interrupt condition
* is met. An interrupt flag will be set and it needs to cleared the
* when exiting the interrupt so the request doesn’t happen after
* returning from the function.
--------------------------------------------------------------------*/
void TimerA0_Handler(void) {
	TA0CCTL0 &= ~CCIFG;                   // Clear the Capture Compare Interrupt Flag
#ifdef PB10
	static uint8_t ticks = 0;             // Holds number of times timer has fired to get bellow 2.5 Hz
	if (ticks < 4) {                      // Check if actual CCV has been reached
		ticks++;
		return;                           // If not, do nothing
	}
	ticks = 0;                            // Else, clear ticks and do actual routine
	if (CYCLING) incrementRGBLED();       // Increment until it is off
	else TA0CCTL0 &= ~TIMER_A_CCTLN_CCIE; // Turn off the timer interrupt when LED is off
#else
	P1OUT ^= BIT0 | BIT7;
#endif
}

inline void incrementRGBLED(void) {
	/* Add one to the 3 bits controlling the LED, resetting to 000 when they reach 111.*/
	P2OUT += ~P2OUT & RGB_BITS ? -RGB_BITS : 1;
}

inline void decrementRGBLED(void) {
	/* Subtract one from the 3 bits controlling the LED, resetting to 111 when they reach 000. */
	P2OUT -= P2OUT & RGB_BITS ? -RGB_BITS : 1;
}
