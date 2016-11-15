/**
 * A library for setting up the timer automatically and exposes easy to use
 * functions to setup up to 4 different PWM signals. This library takes over
 * Timer_A0, so use Timer_A1 for any required timing interrupts, or set the
 * freq of Timer_A0 to your requested frequency through initTA0()
 */

#ifndef MSP_TIMERS
#define MSP_TIMERS

#include <stdint.h>
#include "msp.h"
#include "error.h"

/**
 * @desc Holds the default frequencies available to DCOCLK without calibration
 */
typedef enum {
	FREQ_1_5,
	FREQ_3,
	FREQ_6,
	FREQ_12,
	FREQ_24,
	FREQ_48
} cpuFreq;

/**
 *
 */
typedef enum {
	CCR1 = 1,
	CCR2 = 2,
	CCR3 = 3,
	CCR4 = 4,
} ccrN;

/**
 * @param freqDCO is the desired frequency to run DCOCLK at.
 */
void setCPUFreq(cpuFreq freqDCO);

/**
 * @desc Initializes the RTC
 */
void initRTC(void);

/**
 *
 */
error getTime(uint16_t * hours, uint16_t * minutes, uint16_t * seconds);

/**
 * @param freq is the frequency at which to pulse width modulate the output, use
 *        pwm1() to pwm4() to initiate the modulation
 */
error initTA0(uint32_t freq);

/**
 * Start PWM output based on the frequency setup using the function initTA0().
 * Map a port to PMAP_TA0CCRNA for the PWM signal, where N corresponds to the
 * pwm register you are using
 * @param dutyCycle is a 1 to 100 value that represents the percent time to
 *        stay high. Any value outside this range turns the pwm signal off
 */
void pwm(uint8_t dutyCycle, ccrN reg);

#endif
