#ifndef MSP_SERIAL
#define MSP_SERIAL

#include <stdint.h>
#include "msp.h"
#include "error.h"

/**
 * @desc  Sets up UART on the given eUSCI_A module to the specified baud rate.
 *        Takes into account current DCOCLK freq and uses SMCLK, so make sure
 *        SMCLK source is set to DCOCLK with no prescaler.
 * @param module - Any of EUSCI_A0 to EUSCI_A3.
 * @param baud - The integer baud rate desired. Defaults to 9600 if the
 *               requested rate is not achievable with the current DCOCLK
 *               frequency.
 */
error configUART(EUSCI_A_Type * module, uint32_t baud);

/**
 * @desc  These functions setup a given eUSCI module to be a master or slave.
 *        Masters use the 3 wire setup, slaves use 4 wire. Both have MSB first,
 *        8 bit packets, idle low clocks, and phase shifted reads, so they
 *        happen on the rising edge of the clock. Use GPIO pins for the slave
 *        select in master mode. Clocks from masters are tied to SMCLK divided
 *        by 8, so up to 64MHz freq can be used and SPI clock is below 1MHz.
 * @param module is any of EUSCI_A0_SPI to EUSCI_B3_SPI. Use the appropriate
 *        port mapping.
 */
error configSPIMasterA(EUSCI_A_SPI_Type * module);
error configSPIMasterB(EUSCI_B_SPI_Type * module);
error configSPISlaveA(EUSCI_A_SPI_Type * module);
error configSPISlaveB(EUSCI_B_SPI_Type * module);
error startUART(EUSCI_A_Type * module);
error startSPIA(EUSCI_A_SPI_Type * module);
error startSPIB(EUSCI_B_SPI_Type * module);

#endif
