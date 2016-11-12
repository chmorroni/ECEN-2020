#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include "Libs/uart.h"
#include "Libs/buffer.h"
#include "core_cm4.h"

void main(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
	
}
