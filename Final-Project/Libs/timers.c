#include "timers.h"
#define MAX_TA0_PRESCALER 0xFFFF

/**
 * @desc Initializes the RTC
 * TODO
 */
void initRTC(void) {
	RTC_C->CTL0 = RTC_KEY;               // Unlock register
	RTC_C->CTL0 = RTC_C_CTL0_RDYIE |     // Interrupt every second to update time
	              RTC_KEY;               // Stay unlocked
	RTC_C->CTL13 = RTC_C_CTL13_RDY;      // Calendar mode (0 is reserved)
	RTC_C->CTL0 &= ~RTC_C_CTL0_KEY_MASK; // Relock by clearing key
//	NVIC_EnableIRQ(RTC_C_IRQn);
}

error getTime(uint16_t * hours, uint16_t * minutes, uint16_t * seconds) {
	if (!hours || !minutes || !seconds) return ERR_NULL_PTR;
	*seconds = (RTC_C->TIM0 & RTC_C_TIM0_SEC_MASK) >> RTC_C_TIM0_SEC_OFS;
	*minutes = (RTC_C->TIM0 & RTC_C_TIM0_MIN_MASK) >> RTC_C_TIM0_MIN_OFS;
	*hours   = (RTC_C->TIM1 & RTC_C_TIM1_HOUR_MASK) >> RTC_C_TIM1_HOUR_OFS;
	return ERR_NO;
}

/**
 *
 * @param freqDCO should be one of the below
 *   CS_CTL0_DCORSEL_0 // Nominal DCO Frequency Range (MHz): 1 to 2
 *   CS_CTL0_DCORSEL_1 // Nominal DCO Frequency Range (MHz): 2 to 4
 *   CS_CTL0_DCORSEL_2 // Nominal DCO Frequency Range (MHz): 4 to 8
 *   CS_CTL0_DCORSEL_3 // Nominal DCO Frequency Range (MHz): 8 to 16
 *   CS_CTL0_DCORSEL_4 // Nominal DCO Frequency Range (MHz): 16 to 32
 *   CS_CTL0_DCORSEL_5 // Nominal DCO Frequency Range (MHz): 32 to 64
 */
void setCPUFreq(cpuFreq freqDCO) {
	/* Configure required clocks */
	CS->KEY = 0x695A;           // Unlock CS module for register access
	switch (freqDCO) {
	case FREQ_1_5:              // Setup DCO Clock to Requested frequency
		CS->CTL0 = CS_CTL0_DCORSEL_0;
		break;
	case FREQ_3:
		CS->CTL0 = CS_CTL0_DCORSEL_1;
		break;
	case FREQ_6:
		CS->CTL0 = CS_CTL0_DCORSEL_2;
		break;
	case FREQ_12:
		CS->CTL0 = CS_CTL0_DCORSEL_3;
		break;
	case FREQ_24:
		CS->CTL0 = CS_CTL0_DCORSEL_4;
		break;
	case FREQ_48:

	}
	CS->CTL1 = CS_CTL1_SELA_2 | // ACLK source set to REFOCLK
	           CS_CTL1_SELS_3 | // SMCLK set to DCOCLK
	           CS_CTL1_SELM_3;  // MCLK set to DCOCLK
	CS->KEY = 0;                // lock CS module for register access
}

/**
 * @param freq is the frequency at which to pulse width modulate the output, use
 *        pwm1() to pwm4() to initiate the modulation
 */
error initTA0(uint32_t freq) {
	uint32_t cpuHz = 1500000 * (1 << ((CS->CTL0 & CS_CTL0_DCORSEL_MASK) >> CS_CTL0_DCORSEL_OFS));
	uint32_t minFreq = cpuHz / MAX_TA0_PRESCALER;  // Cannot handle fractions, doesn't matter for PWM frequencies
	if (freq < minFreq) return ERR_PARAM_OUT_OF_BOUNDS;
	TIMER_A0->CCR[0] = 0;                          // Stop timer for config
	TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |      // Use SMCLK
	                TIMER_A_CTL_ID__1 |            // Input divider
	                TIMER_A_CTL_MC__UP;            // Count up to TA0CCR0
	TIMER_A0->CCTL[0] = 0;                         // Defaults look good
	TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_0;    // Output bit, default 0
	TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_0;    // Output bit, default 0
	TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_0;    // Output bit, default 0
	TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_0;    // Output bit, default 0
	TIMER_A0->CTL |= TIMER_A_CTL_CLR;              // Reset the timer
	TIMER_A0->CCR[0] = cpuHz / freq;               // Start timer at 200Hz
	return ERR_NO;
}

void pwm(uint8_t dutyCycle, ccrN reg) {
	if (dutyCycle < 1 || dutyCycle > 100) {            // Out of bounds, default to off
		TIMER_A0->CCTL[reg] = TIMER_A_CCTLN_OUTMOD_0;  // Output bit, default 0
	}
	else if (dutyCycle == 100) {
		TIMER_A0->CCTL[reg] = TIMER_A_CCTLN_OUTMOD_0 | // Output bit, set to 1
		                      TIMER_A_CCTLN_OUT;
	}
	else {
		TIMER_A0->CCTL[reg] = TIMER_A_CCTLN_OUTMOD_3;  // Set / Reset
		TIMER_A0->CCTL[reg] = TIMER_A0->CCR[0] / 100 * dutyCycle;
	}
}
