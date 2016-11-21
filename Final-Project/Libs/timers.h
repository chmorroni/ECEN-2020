/**
 * A library for setting up the timer automatically and exposes easy to use
 * functions to setup up to 4 different PWM signals. This library takes over
 * Timer_A0 for pwm, so use Timer_A1 for any required timing interrupts, or set
 * the freq of Timer_A0 to your requested frequency and don't use pwm if you
 * need low frequencies from it.
 */

#ifndef MSP_TIMERS
#define MSP_TIMERS

#include <stdint.h>
#include "msp.h"
#include "error.h"

/**
 * The requested step size of pwm changes. This affects the frequency timers are
 * initialized to with initTimerA(). Defaults to 100 for 1/100 step sizes.
 */
#ifndef PWM_STEPS
#define PWM_STEPS (1000)
#endif

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
 * The available capture / compare registers. The enum values correspond to
 * the correct indices in the CCR[] and CCTL[] arrays.
 */
typedef enum {
	CCR1 = 1,
	CCR2 = 2,
	CCR3 = 3,
	CCR4 = 4,
} ccrN;

/**
 * @desc  Sets the DCOCLK to the specified frequency, ACLK to REFOCLK, and SMCLK
 *        and MCLK both to DCOCLK.
 * @param freqDCO - The desired frequency to run DCOCLK at.
 */
void setCPUFreq(cpuFreq freqDCO);

/**
 * @desc  Initializes the RTC
 */
void initRTC(void);

/**
 * @desc  Used to read time from the RTC.
 */
inline uint8_t getHours(void);
inline uint8_t getMinutes(void);
inline uint8_t getSeconds(void);

/**
 *  @desc Initializes a timer to the requested frequency. Can go from SMCLK to
 *        0.008 Hz. The timer is put into up mode and CCRN registers are reset.
 *  @param timer - The timer to initialize.
 *  @param freq - The frequency with which the interrupt should fire.
 */
error initTimerA(Timer_A_Type * timer, float freq);

/**
 * @desc  Start PWM output based on the frequency setup using the function
 *        initTimerA() on the TIMER_A0 module. Map a port to PMAP_TA0CCRNA for
 *        the PWM signal, where N corresponds to the pwm register you are using
 *        (selected by ccrN).
 * @param dutyCycle - A 1 to PWM_PRECISION value that represents the time to
 *                    stay high. Any value outside this range turns the pwm
 *                    signal off.
 * @param reg - The ccrN register to use. This corresponds to the output signal
 *              you want to use with port mapping
 */
void pwm(float dutyCycle, ccrN reg);

#endif
