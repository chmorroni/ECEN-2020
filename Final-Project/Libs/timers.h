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
#include "async.h"

/**
 * The requested step size of pwm changes. This affects the frequency timers are
 * initialized to with initTimerA(). Defaults to 100 for 1/100 step sizes.
 */
#ifndef PWM_STEPS
#define PWM_STEPS (1000)
#endif

/**
 * Holds the default frequencies available to DCOCLK without calibration
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
 * Sets the DCOCLK to the specified frequency, ACLK to REFOCLK, and SMCLK and
 * MCLK both to DCOCLK.
 */
void setCPUFreq(cpuFreq freqDCO);

/**
 * Handler for 1 Hz RTC interrupt.
 */
void RTC_C_IRQHandler(void);

/**
 * Initializes the RTC.
 */
void initRTC(void);

/**
 * Used to read time from the RTC after it has been initialized.
 */
inline uint8_t getHours(void);

inline uint8_t getMinutes(void);

inline uint8_t getSeconds(void);

/**
 * Initializes a timer to the requested frequency. Can go from SMCLK frequency
 * to 0.008 Hz. The timer is put into up mode and CCRN registers are reset.
 */
error initTimerA(Timer_A_Type * timer, float freq);

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
void pwm(float dutyCycle, ccrN reg);

/**
 * Initializes timer used for delay function.
 */
void initDelay(void);

/**
 * Delay the running of a function. Soft real time, runs asynchronously, not
 * within interrupt. Takes the delay in increments of 10 microseconds. Goes up
 * to about 100 seconds. Only one delay at a time. Running delay() before the
 * last interval passed to delay() is over is ignored.
 */
void delay(asyncFuncPtr callback, uint32_t decaMicroSecs);

/**
 * Returns True if delay operation in progress.
 */
uint8_t delaying(void);

/**
 * Handler for delay timer.
 */
void T32_INT1_IRQHandler(void);

#endif
