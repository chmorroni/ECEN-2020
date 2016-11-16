/**
 * A library for setting up the timer automatically and exposes easy to use
 * functions to setup up to 4 different PWM signals. This library takes over
 * Timer_A0, so use Timer_A1 for any required timing interrupts, or set the
 * freq of Timer_A0 to your requested frequency through initTA0()
 */

#ifndef MSP_PORT_MAPPING
#define MSP_PORT_MAPPING

#include <stdint.h>
#include "msp.h"
#include "helpers.h"

typedef enum {
	PIN_0,
	PIN_1,
	PIN_2,
	PIN_3,
	PIN_4,
	PIN_5,
	PIN_6,
	PIN_7
} pin;

typedef enum {
	PORT_2,
	PORT_3,
	PORT_7
} port;

void mapPin(port portToMap, pin pinToMap, uint8_t mapping);

#endif
