#include "serial.h"

/**
 * @desc  Sets up UART on the given eUSCI_A module to the specified baud rate.
 *        Takes into account current DCOCLK freq and uses SMCLK, so make sure
 *        SMCLK source is set to DCOCLK with no prescaler.
 * @param module - Any of EUSCI_A0 to EUSCI_A3.
 * @param baud - The integer baud rate desired.
 */
error configUART(EUSCI_A_Type * module, uint32_t baud) {
	float mhz, n, n16, fracn;
	uint32_t intn, brs;
	if (!module) return ERR_NULL_PTR;

	// Calculate current DCO frequency
	mhz = 1.555 * (1 << ((CS->CTL0 & CS_CTL0_DCORSEL_MASK) >> CS_CTL0_DCORSEL_OFS));
	if (baud > mhz * 1000000) return ERR_PARAM_OUT_OF_BOUNDS;

	module->CTLW0 |= EUSCI_A_CTLW0_SWRST;       // Put eUSCI in reset
	module->CTLW0 = EUSCI_A_CTLW0_SSEL__SMCLK | // Use SMCLK
	                EUSCI_A_CTLW0_SWRST;        // Keep eUSCI in reset
	n = mhz * 1000000 / baud;                   // N = freqSMCLK / baud
	n16 = n / 16;
	intn = (uint32_t) (n > 16 ? n16 : n);       // INT(N/16)
	fracn = ((n > 16 ? n16 : n) - intn);        // N - INT(N/16)

	/* Table 22-4 in family datasheet */
	brs = fracn >= 0.9288 ? 0xFE :
	      fracn >= 0.9170 ? 0xFD :
	      fracn >= 0.9004 ? 0xFB :
	      fracn >= 0.8751 ? 0xF7 :
	      fracn >= 0.8572 ? 0xEF :
	      fracn >= 0.8464 ? 0xDF :
	      fracn >= 0.8333 ? 0xBF :
	      fracn >= 0.8004 ? 0xEE :
	      fracn >= 0.7861 ? 0xED :
	      fracn >= 0.7503 ? 0xDD :
	      fracn >= 0.7147 ? 0xBB :
	      fracn >= 0.7001 ? 0xB7 :
	      fracn >= 0.6667 ? 0xD6 :
	      fracn >= 0.6432 ? 0xB6 :
	      fracn >= 0.6254 ? 0xB5 :
	      fracn >= 0.6003 ? 0xAD :
	      fracn >= 0.5715 ? 0x6B :
	      fracn >= 0.5002 ? 0xAA :
	      fracn >= 0.4378 ? 0x55 :
	      fracn >= 0.4286 ? 0x53 :
	      fracn >= 0.4003 ? 0x92 :
	      fracn >= 0.3753 ? 0x52 :
	      fracn >= 0.3575 ? 0x4A :
	      fracn >= 0.3335 ? 0x49 :
	      fracn >= 0.3000 ? 0x25 :
	      fracn >= 0.2503 ? 0x44 :
	      fracn >= 0.2224 ? 0x22 :
	      fracn >= 0.2147 ? 0x21 :
	      fracn >= 0.1670 ? 0x11 :
	      fracn >= 0.1430 ? 0x20 :
	      fracn >= 0.1252 ? 0x10 :
	      fracn >= 0.1001 ? 0x08 :
	      fracn >= 0.0835 ? 0x04 :
	      fracn >= 0.0715 ? 0x02 :
	      fracn >= 0.0529 ? 0x01 : 0;

	if (n > 16) { // Need to use oversampling mode
		// Set clock scaler to INT(N/16)
		module->BRW = intn;
		                // INT([(N/16) – INT(N/16)] × 16
		module->MCTLW = ((uint32_t) (fracn * 16)) << EUSCI_A_MCTLW_BRF_OFS |
		                // Modulation pattern (see table above)
		                brs << EUSCI_A_MCTLW_BRS_OFS |
						// Enable oversampling mode
		                EUSCI_A_MCTLW_OS16;
	}
	else { // Don't need to use oversampling mode
		// Set clock scaler to N
		module->BRW = intn;
		                // Modulation pattern (see table above)
		module->MCTLW = brs << EUSCI_A_MCTLW_BRS_OFS;
	}
	return ERR_NO;
}

/**
 * Does nothing until startSPI() is called on the module, configure port mapping
 * after running config and before running start.
 *
 * @desc  Sets up the given eUSCI_A module to be a SPI master. Uses 4-wire mode,
 *        STE is active low, clock is idle low, shifts data out on falling edge
 *        and reads on rising edge, Clock is tied to the SMCLK and divided by 64
 *        to keep it under 1MHz and give time for the interrupt to retrieve a
 *        new packet from a buffer. Sends MSB first, packets are 8 bits.
 * @param module - A eUSCI_AN module (N being the number of the module) to
 *                 configure.
 */
error configSPIMaster(EUSCI_A_SPI_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI_AN in reset
	module->CTLW0 = EUSCI_A_CTLW0_CKPH |   // Shift on falling edge
	                EUSCI_A_CTLW0_MSB |    // MSB first
	                EUSCI_A_CTLW0_MST |    // Master
	                EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                EUSCI_A_CTLW0_SYNC |   // SPI mode
	                EUSCI_A_CTLW0_SSEL__SMCLK | // Use SMCLK
	                EUSCI_A_CTLW0_STEM |   // STE used for slave
	                EUSCI_A_CTLW0_SWRST;   // Stay in reset
	module->BRW = 0x30;                    // Divide by enough to safely use 48MHz
	module->STATW = 0;                     // Defaults
	return ERR_NO;
}

/**
 * Does nothing until startSPI() is called on the module, configure port mapping
 * after running config and before running start.
 *
 * @desc  Sets up the given eUSCI_A module to be a SPI slave. Uses 4-wire mode,
 *        STE is active low, clock is idle low, shifts data out on falling edge
 *        and reads on rising edge. Sends MSB first, packets are 8 bits.
 * @param module - A eUSCI_AN module (N being the number of the module) to
 *                 configure.
 */
error configSPISlave(EUSCI_A_SPI_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI_AN in reset
	module->CTLW0 = EUSCI_A_CTLW0_CKPH |   // Shift on falling edge
	                EUSCI_A_CTLW0_MSB |    // MSB first
	                EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                EUSCI_A_CTLW0_SYNC |   // SPI mode
	                EUSCI_A_CTLW0_SWRST;   // Stay in reset
	module->STATW = 0;                     // Defaults
	return ERR_NO;
}

/**
 * @desc  Release the given module for operation after calling configUART() and
 *        configuring the required port mapping.
 * @param module - The eUSCI_AN module to to start.
 */
error startUART(EUSCI_A_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Release for operation
	return ERR_NO;
}

/**
 * @desc  Release the given module for operation after calling configSPI() and
 *        configuring the required port mapping.
 * @param module - The eUSCI_AN module to to start.
 */
error startSPI(EUSCI_A_SPI_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Release for operation
	return ERR_NO;
}
