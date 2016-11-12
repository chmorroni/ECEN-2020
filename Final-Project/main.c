#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include "Libs/buffer.h"
#include "core_cm4.h"
#include "Libs/serial.h"

#define CAP_COMP_VAL_200HZ (10000)
#define DUTY_CYCLE (0.5)

void main(void) {
	WDTCTL = WDTPW | WDTHOLD;                      // Stop watchdog timer

	/* Setup the PWM output on P2.6 for the LCD. The ordering is important here,
	 * we stop the timer first, then setup the general control register,
	 * followed by CCTL0 for the pulse rate, then the CCTLn registers to set the
	 * duty cycle. After this we reset the timer and start it counting again by
	 * setting CCR0 to a non-zero value.
	 */
	TIMER_A0->CCR[0] = 0;                          // Stop timer for config
	TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |      // Use SMCLK
	                TIMER_A_CTL_ID__1 |            // Prescale by 8
	                TIMER_A_CTL_MC__UP;            // Count up to TA0CCR0
	TIMER_A0->CCTL[0] = 0;                         // Defaults look good
	TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;    // Set/Reset
	TIMER_A0->CCR[3] = (uint16_t) (DUTY_CYCLE * CAP_COMP_VAL_200HZ);
	TIMER_A0->CTL |= TIMER_A_CTL_CLR;              // Reset the timer
	TIMER_A0->CCR[0] = CAP_COMP_VAL_200HZ;         // Start timer at 200Hz

	/* Setup pin for PWM
	 * Needs to attach to OUT3 of TA0 (See page 147 of specific DS)
	 */
	P2DIR |= BIT6;
	P2SEL1 &= ~BIT6;
	P2SEL0 |= BIT6;
	
	__enable_interrupts();

	while(1);
}
