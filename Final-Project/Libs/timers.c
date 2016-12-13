#include "timers.h"
#include "async.h"
#include "helpers.h"
#define TA0_PRESCALER_MAX 64
#define UINT16_T_MAX (0xFFFF)
#define RTC_KEY ((uint16_t) 0xA500)

typedef struct {
	uint16_t hours;
	uint16_t minutes;
	uint16_t seconds;
} time;

static time current = {0};
static asyncFuncPtr delayCallback = NULL;

/**
 * Sets the DCOCLK to the specified frequency, ACLK to REFOCLK, and SMCLK and
 * MCLK both to DCOCLK.
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
	CS->CTL1 = CS_CTL1_SELB |   // BCLK set to REFOCLK
	           CS_CTL1_SELA_2 | // ACLK source set to REFOCLK
	           CS_CTL1_SELS_3 | // SMCLK set to DCOCLK
	           CS_CTL1_SELM_3;  // MCLK set to DCOCLK
	CS->KEY = 0;                // lock CS module for register access
}

/**
 * Handler for 1 Hz RTC interrupt.
 */
void RTC_C_IRQHandler(void) {
	RTC_C->IV; // Clear interrupt
	current.seconds = (RTC_C->TIM0 & RTC_C_TIM0_SEC_MASK)  >> RTC_C_TIM0_SEC_OFS;
	current.minutes = (RTC_C->TIM0 & RTC_C_TIM0_MIN_MASK)  >> RTC_C_TIM0_MIN_OFS;
	current.hours   = (RTC_C->TIM1 & RTC_C_TIM1_HOUR_MASK) >> RTC_C_TIM1_HOUR_OFS;
}

/**
 * Initializes the RTC.
 */
void initRTC(void) {
	RTC_C->CTL0 = RTC_KEY;               // Unlock register
	RTC_C->CTL0 = RTC_C_CTL0_RDYIE |     // Interrupt every second to update time
	              RTC_KEY;               // Stay unlocked
	RTC_C->CTL13 = RTC_C_CTL13_MODE;     // Calendar mode (0 is reserved)
	RTC_C->CTL0 &= ~RTC_C_CTL0_KEY_MASK; // Relock by clearing key
	NVIC_EnableIRQ(RTC_C_IRQn);
}

/**
 * Used to read time from the RTC after it has been initialized.
 */
inline uint8_t getHours(void) {
	return current.hours;
}

inline uint8_t getMinutes(void) {
	return current.minutes;
}

inline uint8_t getSeconds(void) {
	return current.seconds;
}

/**
 * Initializes a timer to the requested frequency. Can go from SMCLK frequency
 * to 0.008 Hz. The timer is put into up mode and CCRN registers are reset.
 */
error initTimerA(Timer_A_Type * timer, float freq) {
	float clkHz = 1500000 * (1 << ((CS->CTL0 & CS_CTL0_DCORSEL_MASK) >> CS_CTL0_DCORSEL_OFS));
	float minFreq = clkHz / UINT16_T_MAX;
	uint16_t ctl0ID = TIMER_A_CTL_ID__1;
	uint16_t ex0ID = TIMER_A_EX0_IDEX__1;
	uint16_t clk = TIMER_A_CTL_SSEL__SMCLK;
	if (!timer) return ERR_NULL_PTR;
	if (freq > clkHz) return ERR_PARAM_OUT_OF_BOUNDS;
	// Runs through possible configurations of input dividers and clock sources
	// until the frequency can be achieved. Errors out if it fails.
	if (freq < minFreq) {
		ctl0ID = TIMER_A_CTL_ID__8;
		clkHz /= 8;
		minFreq /= 8;
		if (freq < minFreq) {
			ex0ID = TIMER_A_EX0_IDEX__8;
			clkHz /= 8;
			minFreq /= 8;
		}
		if (freq < minFreq) {
			clk = TIMER_A_CTL_SSEL__ACLK; // Can't be achieved with SMCLK
			clkHz = 32000; // Default freq for low frequency clock.
			minFreq = clkHz / UINT16_T_MAX;
			ctl0ID = TIMER_A_CTL_ID__1;
			ex0ID = TIMER_A_EX0_IDEX__1;
			if (freq > clkHz) return ERR_PARAM_OUT_OF_BOUNDS;
			if (freq < minFreq) {
				ctl0ID = TIMER_A_CTL_ID__8;
				clkHz /= 8;
				minFreq /= 8;
				if (freq < minFreq) {
					ex0ID = TIMER_A_EX0_IDEX__8;
					clkHz /= 8;
					minFreq /= 8;
					if (freq < minFreq) return ERR_PARAM_OUT_OF_BOUNDS;
				}
			}
		}
	}
	timer->CCR[0] = 0;                       // Stop timer for config
	timer->CTL = clk |                       // Use SMCLK
	             ctl0ID |                    // Input divider
	             TIMER_A_CTL_MC__UP;         // Count up to TA0CCR0
	timer->EX0 = ex0ID;
	timer->CCTL[0] = 0;                      // Defaults look good
	timer->CCTL[1] = TIMER_A_CCTLN_OUTMOD_0; // Output bit, default 0
	timer->CCTL[2] = TIMER_A_CCTLN_OUTMOD_0; // Output bit, default 0
	timer->CCTL[3] = TIMER_A_CCTLN_OUTMOD_0; // Output bit, default 0
	timer->CCTL[4] = TIMER_A_CCTLN_OUTMOD_0; // Output bit, default 0
	timer->CTL |= TIMER_A_CTL_CLR;           // Reset the timer
	timer->CCR[0] = clkHz / freq;            // Start timer
	return ERR_NO;
}

/**
 * Start PWM output based on the frequency setup using the function initTimerA()
 * on the TIMER_A0 module. Map a port to PMAP_TA0CCRNA for the PWM signal, where
 * N corresponds to the pwm register you are using (selected by ccrN).
 * dutyCycle - A 0 to 1 value that represents the % time  to
 *             stay high. Any value outside this range turns the pwm
 *             signal off.
 * reg - The ccrN register to use. This corresponds to the output signal
 *       you want to use with port mapping
 */
void pwm(float dutyCycle, ccrN reg) {
	if (dutyCycle <= 0) {     // Out of bounds / off -> default to off
		TIMER_A0->CCTL[reg] = TIMER_A_CCTLN_OUTMOD_0; // Output bit, default 0
	}
	else if (dutyCycle >= 1) {
		TIMER_A0->CCTL[reg] = TIMER_A_CCTLN_OUTMOD_0 | // Output bit
		                      TIMER_A_CCTLN_OUT;       // Set OUTN signal high
	}
	else {
		TIMER_A0->CCTL[reg] = TIMER_A_CCTLN_OUTMOD_7;  // Set / Reset
		TIMER_A0->CCR[reg] = (uint16_t) (TIMER_A0->CCR[0] * dutyCycle);
	}
}

/**
 * Initializes timer used for delay function.
 */
void initDelay(void) {
	initAsync();
	NVIC_EnableIRQ(T32_INT1_IRQn);
}

/**
 * Delay the running of a function. Soft real time, runs asynchronously, not
 * within interrupt. Takes the delay in increments of 10 microseconds. Goes up
 * to about 100 seconds. Only one delay at a time. Running delay() before the
 * last interval passed to delay() is over is ignored.
 */
void delay(asyncFuncPtr callback, uint32_t decaMicroSecs) {
	if (delayCallback) return;
	TIMER32_1->CONTROL &= ~TIMER32_CONTROL_ENABLE;
	TIMER32_1->LOAD = 15 * decaMicroSecs * (1 << ((CS->CTL0 & CS_CTL0_DCORSEL_MASK) >> CS_CTL0_DCORSEL_OFS));
	delayCallback = callback;
	TIMER32_1->CONTROL = TIMER32_CONTROL_ONESHOT | // Run once
			             TIMER32_CONTROL_SIZE |    // 32 bit mode for timer
						 TIMER32_CONTROL_MODE |    // Use LOAD register for number
						 TIMER32_CONTROL_ENABLE |  // Enable timer
						 TIMER32_CONTROL_IE;       // Enable interrupt
}

/**
 * Returns True if delay operation in progress.
 */
uint8_t delaying(void) {
	return !!delayCallback;
}

/**
 * Handler for delay timer.
 */
void T32_INT1_IRQHandler(void) {
	if (!delayCallback) return;
	runAsync(delayCallback);
	delayCallback = NULL;
	TIMER32_1->INTCLR = 1;
}
