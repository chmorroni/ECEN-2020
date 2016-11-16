#include "timers.h"
#define TA0_PRESCALER_MAX 64
#define UINT16_T_MAX (0xFFFF)
#define PWM_PRECISION (100)
#define RTC_KEY ((uint16_t) 0xA500)

/**
 * @desc Initializes the RTC
 * TODO
 */
void initRTC(void) {
	RTC_C->CTL0 = RTC_KEY;               // Unlock register
	RTC_C->CTL0 = RTC_C_CTL0_RDYIE |     // Interrupt every second to update time
	              RTC_KEY;               // Stay unlocked
	RTC_C->CTL13 = RTC_C_CTL13_MODE;      // Calendar mode (0 is reserved)
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
		CS->CTL0 = CS_CTL0_DCORSEL_5;
	}
	CS->CTL1 = CS_CTL1_SELA_2 | // ACLK source set to REFOCLK
	           CS_CTL1_SELS_3 | // SMCLK set to DCOCLK
	           CS_CTL1_SELM_3;  // MCLK set to DCOCLK
	CS->KEY = 0;                // lock CS module for register access
}

error initTimerA(Timer_A_Type * timer, float freq) {
	float clkHz = 1500000 * (1 << ((CS->CTL0 & CS_CTL0_DCORSEL_MASK) >> CS_CTL0_DCORSEL_OFS));
	float minFreq = clkHz / UINT16_T_MAX;
	uint16_t ctl0ID = TIMER_A_CTL_ID__1;
	uint16_t ex0ID = TIMER_A_EX0_IDEX__1;
	uint16_t clk = TIMER_A_CTL_SSEL__SMCLK;
	if (!timer) return ERR_NULL_PTR;
	if (freq > clkHz) return ERR_PARAM_OUT_OF_BOUNDS;
	if (freq < minFreq * PWM_PRECISION) {
		ctl0ID = TIMER_A_CTL_ID__8;
		clkHz /= 8;
		minFreq /= 8;
	}
	if (freq < minFreq * PWM_PRECISION) {
		ex0ID = TIMER_A_EX0_IDEX__8;
		clkHz /= 8;
		minFreq /= 8;
	}
	if (freq < minFreq * PWM_PRECISION) {
		clk = TIMER_A_CTL_SSEL__ACLK;
		clkHz = 32000;
		minFreq = clkHz / UINT16_T_MAX;
		ctl0ID = TIMER_A_CTL_ID__1;
		ex0ID = TIMER_A_EX0_IDEX__1;
		if (freq > clkHz) return ERR_PARAM_OUT_OF_BOUNDS;
		if (freq < minFreq * PWM_PRECISION) {
			ctl0ID = TIMER_A_CTL_ID__8;
			clkHz /= 8;
			minFreq /= 8;
		}
		if (freq < minFreq * PWM_PRECISION) {
			ex0ID = TIMER_A_EX0_IDEX__8;
			clkHz /= 8;
			minFreq /= 8;
		}
	}
	timer->CCR[0] = 0;                          // Stop timer for config
	timer->CTL = clk |                          // Use SMCLK
	             ctl0ID |                       // Input divider
	             TIMER_A_CTL_MC__UP;            // Count up to TA0CCR0
	timer->EX0 = ex0ID;
	timer->CCTL[0] = 0;                         // Defaults look good
	timer->CCTL[1] = TIMER_A_CCTLN_OUTMOD_0;    // Output bit, default 0
	timer->CCTL[2] = TIMER_A_CCTLN_OUTMOD_0;    // Output bit, default 0
	timer->CCTL[3] = TIMER_A_CCTLN_OUTMOD_0;    // Output bit, default 0
	timer->CCTL[4] = TIMER_A_CCTLN_OUTMOD_0;    // Output bit, default 0
	timer->CTL |= TIMER_A_CTL_CLR;              // Reset the timer
	timer->CCR[0] = clkHz / freq;               // Start timer at 200Hz
	if (freq < minFreq * PWM_PRECISION) return ERR_PARAM_OUT_OF_BOUNDS;
	return ERR_NO;
}

void pwm(uint8_t dutyCycle, ccrN reg) {
	if (dutyCycle < 1 || dutyCycle > PWM_PRECISION) {            // Out of bounds, default to off
		TIMER_A0->CCTL[reg] = TIMER_A_CCTLN_OUTMOD_0;  // Output bit, default 0
	}
	else if (dutyCycle == PWM_PRECISION) {
		TIMER_A0->CCTL[reg] = TIMER_A_CCTLN_OUTMOD_0 | // Output bit, set to 1
		                      TIMER_A_CCTLN_OUT;
	}
	else {
		TIMER_A0->CCTL[reg] = TIMER_A_CCTLN_OUTMOD_7;  // Set / Reset
		TIMER_A0->CCR[reg] = TIMER_A0->CCR[0] / PWM_PRECISION * dutyCycle;
	}
}
