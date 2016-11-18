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
 * @param baud - The integer baud rate desired.
 */
error configUART(EUSCI_A_Type * module, uint32_t baud);

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
error configSPIMaster(EUSCI_A_SPI_Type * module);

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
error configSPISlave(EUSCI_A_SPI_Type * module);

/**
 * @desc  Release the given module for operation after calling configUART() and
 *        configuring the required port mapping.
 * @param module - The eUSCI_AN module to to start.
 */
error startUART(EUSCI_A_Type * module);

/**
 * @desc  Release the given module for operation after calling configSPI() and
 *        configuring the required port mapping.
 * @param module - The eUSCI_AN module to to start.
 */
error startSPI(EUSCI_A_SPI_Type * module);

#endif
