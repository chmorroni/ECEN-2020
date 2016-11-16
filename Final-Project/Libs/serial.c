#include "serial.h"

error configUART(EUSCI_A_Type * module, uint32_t baud) {
	float mhz, n, n16, fracn;
	uint32_t intn, brs;
	if (!module) return ERR_NULL_PTR;

	mhz = 1.5 * (1 << ((CS->CTL0 & CS_CTL0_DCORSEL_MASK) >> CS_CTL0_DCORSEL_OFS));
	if (baud > mhz * 1000000) return ERR_PARAM_OUT_OF_BOUNDS;

	module->CTLW0 |= EUSCI_A_CTLW0_SWRST;       // Put eUSCI in reset
	module->CTLW0 = EUSCI_A_CTLW0_SSEL__SMCLK | // Use SMCLK
	                EUSCI_A_CTLW0_SWRST;        // Keep eUSCI in reset

	if (!baud) baud = 9600;                     // Default to 9600
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

	if (n > 16) {                                                              // Need to use oversampling mode
		module->BRW = intn;                                                  // Set clock scaler to INT(N/16)
		module->MCTLW = ((uint32_t) (fracn * 16)) << EUSCI_A_MCTLW_BRF_OFS | // INT([(N/16) – INT(N/16)] × 16
		                  brs << EUSCI_A_MCTLW_BRS_OFS |                       // Modulation pattern (see table above)
		                  EUSCI_A_MCTLW_OS16;                                  // Use modulator
	}
	else {                                              // Don't need to use oversampling mode
		module->BRW = intn;                           // Set clock scaler to N
		module->MCTLW = brs << EUSCI_A_MCTLW_BRS_OFS; // Modulation pattern (see table above)
	}
	return ERR_NO;
}

error configSPIMasterA(EUSCI_A_SPI_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI in reset
	module->CTLW0 = EUSCI_A_CTLW0_CKPH |   // phase = 1 to shift on falling edge
	                EUSCI_A_CTLW0_MSB |    // MSB first
	                EUSCI_A_CTLW0_MST |    // Master
	                EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                EUSCI_A_CTLW0_SYNC |   // SPI mode
	                EUSCI_A_CTLW0_SSEL__SMCLK |
	                EUSCI_A_CTLW0_STEM |   // STE used for slave
	                EUSCI_A_CTLW0_SWRST;   // STE used for slave
	module->BRW = 256;                    // Divide by enough to safely use 48MHz
	module->STATW = 0;                     // Defaults
	return ERR_NO;
}
error configSPIMasterB(EUSCI_B_SPI_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI_A1 in reset
	module->CTLW0 = EUSCI_A_CTLW0_CKPH |   // phase = 1 to shift on falling edge
	                EUSCI_A_CTLW0_MSB |    // MSB first
	                EUSCI_A_CTLW0_MST |    // Master
	                EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                EUSCI_A_CTLW0_SYNC |   // SPI mode
	                EUSCI_A_CTLW0_SSEL__SMCLK |
	                EUSCI_A_CTLW0_STEM |   // STE used for slave
	                EUSCI_A_CTLW0_SWRST;   // STE used for slave
	module->BRW = 256;                    // Divide by enough to safely use 48MHz
	module->STATW = 0;                     // Defaults
	return ERR_NO;
}

error configSPISlaveA(EUSCI_A_SPI_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI_A2 in reset
	module->CTLW0 = EUSCI_A_CTLW0_CKPH |
	                EUSCI_A_CTLW0_MSB |    // MSB first
	                EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                EUSCI_A_CTLW0_SYNC |   // SPI mode
	                EUSCI_A_CTLW0_SWRST;   // STE used for slave
	module->STATW = 0;                     // Defaults
	return ERR_NO;
}
error configSPISlaveB(EUSCI_B_SPI_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI_A2 in reset
	module->CTLW0 = EUSCI_A_CTLW0_CKPH |
	                EUSCI_A_CTLW0_MSB |    // MSB first
	                EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                EUSCI_A_CTLW0_SYNC |   // SPI mode
	                EUSCI_A_CTLW0_SWRST;   // STE used for slave
	module->STATW = 0;                     // Defaults
	return ERR_NO;
}

error startUART(EUSCI_A_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Release for operation
	return ERR_NO;
}

error startSPIA(EUSCI_A_SPI_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Release for operation
	return ERR_NO;
}
error startSPIB(EUSCI_B_SPI_Type * module) {
	if (!module) return ERR_NULL_PTR;
	module->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; // Release for operation
	return ERR_NO;
}
