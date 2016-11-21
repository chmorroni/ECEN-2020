#include <stdint.h>
#include "msp.h"
#include "core_cm4.h"
#include "Libs/nrf.h"
#define BUFF_TYPE nrfCmd
#include "Libs/buffer.h"
#include "Libs/serial.h"
#include "Libs/error.h"
#include "Libs/helpers.h"
#define PWM_STEPS (1000)
#include "Libs/timers.h"
#include "Libs/logging.h"
#include "Libs/portMapping.h"

#define TRANSMITTER

#define SPEED_OFS    (2)
#define CMD_OFS      SPEED_OFS
#define CMD_ENCODING (0x03)


typedef enum {
	CMD_STATE,         // Used to indicate that a state should be sent, not a command

	CMD_NOP,           // No op. Could be used to test for connection.

	CMD_LIGHTS_EN,     // Turn on all lights
	CMD_LIGHTS_DI,     // Turn off all lights
	CMD_LIGHTS_BLINK,  // Blink lights

	CMD_MOVE_EN,       // Enable movement
	CMD_MOVE_DS,       // Disable movement

	CMD_MODE_BREAK_EN, // Enable breaking on stop
	CMD_MODE_BREAK_DS, // Disable breaking on stop
	CMD_MODE_FAST_EN,  // Enable max top speed
	CMD_MODE_FAST_DS,  // Disable max top speed
	CMD_MODE_SAFE_EN,  // Enable sensors
	CMD_MODE_SAFE_DS,  // Disable sensors
} command;

typedef enum {
	DIR_STRAIT,
	DIR_RIGHT,
	DIR_LEFT
} direction;

typedef struct {
	int8_t speed;
	direction dir;
	command cmd;
} packet;






/**
 *  Main code for transmitter
 */
#ifdef TRANSMITTER

/**
 *  Useful macros
 */
#define MOSI_SIZE (8)
#define MISO_SIZE (16)

/**
 *  Type definitions
 */
typedef enum {
	MODE_SETUP,             // Changing settings
	MODE_NORMAL,            // Controlling car
	MODE_RANGE_TEST,        // Expecting packet loss, displaying with LED
	MODE_CONNECTION_DROPPED // Lost connection unexpectedly
} opMode;

/**
 *  Global vars
 */
#define GET_NRF_STATUS_FLAG (0x80)
#define STATUS_READ_FLAG    (0x40)
#define BREAK_MODE_FLAG     (0x20)
#define FAST_MODE_FLAG      (0x10)
#define SAFE_MODE_FLAG      (0x08)
#define PACKET_DROP_FLAG    (0x04)
#define SEND_FLAG           (0x02)
#define STATUS_REPORT_FLAG  (0x01)
static uint8_t flags = 0;
static uint8_t nrfStatus = 0;
static uint8_t packetsSent = 0;
static uint8_t droppedPackets = 0;
static packet nextPacket;
static Buff mosi = {0};
static Buff miso = {0};
static struct {
	float brightness;
} prefs = {0.001};
static opMode mode = MODE_NORMAL;
//static float sines[] = {0, -0.125}; // TODO


/**
 * Helper functions
 */

void printHelp(void);
void printInvalidCommand(uint8_t received);
uint8_t encodePacket(packet pac);
inline void requestModeChange(opMode currentMode, opMode newMode);
inline void forceModeChange(opMode newMode);
inline void pauseStatusReports(void);
inline void resumeStatusReports(void);
void endRangeTestYN(char received);
void commandHandler(uint8_t received);
inline void waitForSPI(void);
void nrfSend(void);
void clearNRFStatus(void);
void updateCREE(void);

/**
 * Setup functions
 */

void setupNRF(void);
void setupCREE(void);


void main(void) {
	char str[21]; // Enough for 2^64 + '\0' as well as 0xFFFFFF... + '\0'
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog timer

	setCPUFreq(FREQ_3);

	configLogging(EUSCI_A0, 115200, 32);
	P1->SEL0 |= BIT2 | BIT3;                    // 2 = RX 3 = TX
	P1->SEL1 &= ~(BIT2 | BIT3);                 // Set to primary function (01)
	startLogging(EUSCIA0_IRQn);
	enableTimestamps();
	enableCommands(&commandHandler);

	setupCREE();

	// 1 Hz logging interrupt
	initTimerA(TIMER_A1, 1);
	TIMER_A1->CTL |= TIMER_A_CTL_IE;
	NVIC_EnableIRQ(TA1_N_IRQn);

	// 20Hz state packet sending interrupt
	initTimerA(TIMER_A2, 1);
	TIMER_A2->CTL |= TIMER_A_CTL_IE;
	NVIC_EnableIRQ(TA2_N_IRQn);

	setupNRF();

	__enable_interrupts();

	while(1) {
		updateCREE();
		// A switch with while loops allows a single enter and exit spot, easy
		// and instant changing of modes, and minimal mode checking overhead.
		switch (mode) {
		case MODE_SETUP:
			pauseStatusReports();
			while (mode == MODE_SETUP); // TODO ...
			break;
		case MODE_NORMAL:
			resumeStatusReports();
			while (mode == MODE_NORMAL) {

				if (nrfStatus & NRF_STATUS_TX_DS) {
					clearNRFStatus();
				}

				if (flags & STATUS_REPORT_FLAG) {
					log("Status Report:", SEND_ASYNC);
					printString("            Packets sent:    ", SEND_ASYNC);
					uIntToStr(packetsSent, str, 0);
					printString(str, SEND_ASYNC);
					printNewline(SEND_ASYNC);
					printString("            Dropped packets: ", SEND_ASYNC);
					uIntToStr(droppedPackets, str, 0);
					printString(str, SEND_ASYNC);
					printNewline(SEND_ASYNC);
					printString("            NRF status:      ", SEND_ASYNC);
					uInt8ToHexStr(nrfStatus, str);
					printString(str, SEND_ASYNC);
					printNewline(SEND_ASYNC);
					printNewline(SEND_ASYNC);
					packetsSent = 0;
					droppedPackets = 0;
					flags &= ~STATUS_REPORT_FLAG;
				}

				if (flags & SEND_FLAG) {
					waitForSPI();
					addToBuff(&mosi, NRF_W_TX_PAYLOAD);
					addToBuff(&mosi, encodePacket(nextPacket));
					nrfSend();
					packetsSent++;
					nextPacket.dir = DIR_STRAIT;
					nextPacket.speed = 0;
					flags &= ~SEND_FLAG;
				}

				if (flags & PACKET_DROP_FLAG) {
					droppedPackets++;
					log("ERROR: Packet dropped.", SEND_ASYNC);
					requestModeChange(MODE_NORMAL, MODE_CONNECTION_DROPPED);
				}

				if (flags & GET_NRF_STATUS_FLAG){
					// NRF sent a package or dropped one, need to check
					waitForSPI();
					addToBuff(&mosi, NRF_NOP);
					nrfSend();
					flags &= ~GET_NRF_STATUS_FLAG;
				}
			}
			break;
		case MODE_RANGE_TEST:
			pauseStatusReports();
			while (mode == MODE_RANGE_TEST) {
				if (flags & PACKET_DROP_FLAG) {
					pwm(prefs.brightness, CCR1);
					pwm(0, CCR2);
					pwm(0, CCR3);
				}
				else {
					pwm(0, CCR1);
					pwm(prefs.brightness, CCR2);
					pwm(0, CCR3);
				}
				clearNRFStatus();
				nextPacket.cmd = CMD_NOP;
				waitForSPI();
				addToBuff(&mosi, NRF_W_TX_PAYLOAD);
				addToBuff(&mosi, encodePacket(nextPacket));
				nrfSend();
			}
			break;
		case MODE_CONNECTION_DROPPED:
			pauseStatusReports();
			while (mode == MODE_CONNECTION_DROPPED) {
				if (nrfStatus & NRF_STATUS_MAX_RT) {
					log("Reseting NRF...", SEND_ASYNC);
					clearNRFStatus();
				}
				else {
					log("NRF reset. Resuming normal operation", SEND_ASYNC);
					requestModeChange(MODE_CONNECTION_DROPPED, MODE_NORMAL); // Fixed
				}
			}
			break;
		}
	}
}

/**
 * Helper functions
 */

void printHelp(void) {
	printWrap("Please choose one of the following single character commands:", 0, 0, 0, SEND_INTERRUPT_SAFE);
	printNewline(SEND_INTERRUPT_SAFE);
	printWrap("d - Disable the transmitter.", 4, 8, 0, SEND_INTERRUPT_SAFE);
	printWrap("e - Enable the transmitter.", 4, 8, 0, SEND_INTERRUPT_SAFE);
	printWrap("i - Print system info. This includes the last sent packet, the last receive from the NRF, and the current sensor readings", 4, 8, 0, SEND_INTERRUPT_SAFE);
	printWrap("h - Re-print this help message.", 4, 8, 0, SEND_INTERRUPT_SAFE);
	printWrap("p - Change preferences. Such as the intensity of the LED, the frequency of \npacket exchange, etc.", 4, 8, 0, SEND_INTERRUPT_SAFE);
	printWrap("r - Range test, send NOP codes and report if they succeed or fail with the \ngreen and red LEDs, respectively.", 4, 8, 0, SEND_INTERRUPT_SAFE);
	printWrap("s - Send command. You can send either a command or state. For a command, \nenter 0 to 63, for a state, you will be asked for a speed from 1 to 100, a direction to turn in, and the copies of the packet to send, at the \ncurrent packet rate.", 4, 8, 0, SEND_INTERRUPT_SAFE);
	printNewline(SEND_INTERRUPT_SAFE);
}

void printInvalidCommand(uint8_t received) {
	char str[5];
	printString("That command is not registered. You entered: '", SEND_INTERRUPT_SAFE);
	// Printable characters
	if (received >= 0x20 && received <= 0x7D) printChar(received, SEND_INTERRUPT_SAFE);
	printString("' (ASCII code = ", SEND_INTERRUPT_SAFE);
	uInt8ToHexStr(received, str);
	printString(str, SEND_INTERRUPT_SAFE);
	printChar(')', SEND_INTERRUPT_SAFE);
	printNewline(SEND_INTERRUPT_SAFE);
	printHelp();
}

uint8_t encodePacket(packet pac) {
	return pac.cmd ? pac.cmd << CMD_OFS | CMD_ENCODING :
	                 pac.speed << SPEED_OFS | pac.dir;
}

inline void requestModeChange(opMode currentMode, opMode newMode) {
	__disable_interrupts(); // Makes sure a forced change sticks in an ISR, as it is either rejected here or takes effect immediately after this function.
	if (mode == currentMode) mode = newMode;
	__enable_interrupts();
}

inline void forceModeChange(opMode newMode) {
	mode = newMode;
}

inline void pauseStatusReports(void) {
	TIMER_A2->CTL &= ~TIMER_A_CTL_IE;
}

inline void resumeStatusReports(void) {
	TIMER_A2->CTL |= TIMER_A_CTL_IE;
}

void endRangeTestYN(char received) {
	if (received == 'y' || received == 'Y') {
		printWrap("Range Test Mode deactivated. Resuming normal transmission.", 0, 0, 0, SEND_INTERRUPT_SAFE);
		forceModeChange(MODE_NORMAL);
	}
	else {
		printWrap("Continuing in Range Test Mode.", 0, 0, 0, SEND_INTERRUPT_SAFE);
		forceModeChange(MODE_RANGE_TEST);
	}
}

void commandHandler(uint8_t received) {
	char str[21]; // Enough for 2^64 + '\0' as well as 0xFFFFFF... + '\0'
	switch (received) {
	case 'd':
		// Disable
		break;
	case 'e':
		// Enable
		break;
	case 'i':
		log("Info:", SEND_INTERRUPT_SAFE);
		printString("            Packets sent:    ", SEND_INTERRUPT_SAFE);
		uIntToStr(packetsSent, str, 0);
		printString(str, SEND_INTERRUPT_SAFE);
		printNewline(SEND_INTERRUPT_SAFE);
		printString("            Dropped packets: ", SEND_INTERRUPT_SAFE);
		uIntToStr(packetsSent, str, 0);
		printString(str, SEND_INTERRUPT_SAFE);
		printNewline(SEND_INTERRUPT_SAFE);
		printString("            NRF status:      ", SEND_INTERRUPT_SAFE);
		uInt8ToHexStr(nrfStatus, str);
		printString(str, SEND_INTERRUPT_SAFE);
		printNewline(SEND_INTERRUPT_SAFE);
		printNewline(SEND_INTERRUPT_SAFE);
		break;
	case 'h':
		printHelp();
		break;
	case 'p':
		// Preferences
		break;
	case 'r':
		if (mode != MODE_RANGE_TEST) {
			printWrap("Range Test Mode activated. The LED will glow green when a transmitted package is acknowledged and red when it is not.", 0, 0, 0, SEND_INTERRUPT_SAFE);
			forceModeChange(MODE_RANGE_TEST);
		}
		else {
			if (flags & PACKET_DROP_FLAG) {
				printWrap("Not in range of car. Are you sure you want to deactivate Range Test Mode? (y|n)", 0, 0, 0, SEND_INTERRUPT_SAFE);
				getChar(&endRangeTestYN);
			}
			else endRangeTestYN('Y');
		}
		break;
	case 's':
		// Get speed
		// Get turn direction
		// Get number to send
		// Send command
		break;
	default:
		printInvalidCommand(received);
		break;
	}
}


inline void waitForSPI(void) {
	while (EUSCI_A1_SPI->STATW & EUSCI_A_STATW_BUSY);
	flags &= ~STATUS_READ_FLAG;
	__disable_interrupts();
	P5->OUT &= ~BIT7; // Disable transmitter
}

void nrfSend(void) {
	P5->OUT |= BIT7; // Enable transmitter
	EUSCI_A1_SPI->IE |= EUSCI_A_IE_TXIE;
	__enable_interrupts();
}

void clearNRFStatus(void) {
	// Clear interrupt flags
	waitForSPI();
	addToBuff(&mosi, NRF_W_REGISTER(NRF_STATUS_ADDR));
	addToBuff(&mosi, NRF_STATUS_RX_DR |
			         NRF_STATUS_TX_DS |
					 NRF_STATUS_MAX_RT |
					 NRF_STATUS_RX_P_NO);
	nrfSend();
}

void updateCREE(void) {

	if (mode == MODE_SETUP || mode == MODE_CONNECTION_DROPPED) {
		TIMER_A3->CTL |= TIMER_A_CTL_IE;   // LED should pulse
	}
	else TIMER_A3->CTL &= ~TIMER_A_CTL_IE; // LED shouldn't pulse

	if (mode == MODE_NORMAL) {
		pwm(flags & BREAK_MODE_FLAG ? prefs.brightness : 0, CCR1);
		pwm(flags & FAST_MODE_FLAG  ? prefs.brightness : 0, CCR2);
		pwm(flags & SAFE_MODE_FLAG  ? prefs.brightness : 0, CCR3);
	}

	if (mode == MODE_RANGE_TEST) {
		pwm(0, CCR1);
		pwm(prefs.brightness, CCR2); // Start at green
		pwm(0, CCR3);
	}
}

/**
 * Setup functions
 */

void setupNRF(void) {
	initBuff(&mosi, MOSI_SIZE);
	initBuff(&miso, MISO_SIZE);

	// SPI to nrf
	configSPIMaster(EUSCI_A1_SPI);
	mapPin(PORT_2, PIN_4, PMAP_UCA1STE);
	mapPin(PORT_2, PIN_5, PMAP_UCA1CLK);
	mapPin(PORT_2, PIN_6, PMAP_UCA1SIMO);
	mapPin(PORT_2, PIN_7, PMAP_UCA1SOMI);
	startSPI(EUSCI_A1_SPI);
	EUSCI_A1_SPI->IE |= EUSCI_A_IE_TXIE | EUSCI_A_IE_RXIE;
	NVIC_EnableIRQ(EUSCIA1_IRQn);

	// CE
	P5->OUT &= ~BIT7;
	P5->DIR |= BIT7;
	// IRQ
	P5->DIR &= ~BIT6;
	P5->IFG = 0;
	P5->IES |= BIT6;            // Interrupt fires on high to low transition
	P5->IE |= BIT6;
	NVIC_EnableIRQ(PORT5_IRQn); // Register port 1 interrupts with NVIC

	clearNRFStatus();

	waitForSPI();
	addToBuff(&mosi, NRF_W_REGISTER(NRF_CONFIG_ADDR));
	addToBuff(&mosi, NRF_CONFIG_RX_DR_IIE |
			         NRF_CONFIG_PWR_UP |
			         NRF_CONFIG_PRIM_TX);
	nrfSend();

	waitForSPI();
	addToBuff(&mosi, NRF_W_REGISTER(NRF_RF_SETUP_ADDR));
	addToBuff(&mosi, NRF_RF_SETUP_250KBPS |
	                 NRF_RF_SETUP_RF_PWR_0);
	nrfSend();

	waitForSPI();
	addToBuff(&mosi, NRF_W_REGISTER(NRF_FEATURE_ADDR));
	addToBuff(&mosi, NRF_FEATURE_EN_DYN_ACK);
	nrfSend();

	P5->OUT |= BIT7; // Enable transmitter
}

void setupCREE(void) {
	// PWM timer
	initTimerA(TIMER_A0, 1000);
	// PWM Pins
	P2->DIR |= BIT0 | BIT1 | BIT2;
	mapPin(PORT_2, PIN_0, PMAP_TA0CCR1A);
	mapPin(PORT_2, PIN_1, PMAP_TA0CCR2A);
	mapPin(PORT_2, PIN_2, PMAP_TA0CCR3A);
	pwm(0, CCR1);
	pwm(0, CCR2);
	pwm(0, CCR3);

	// 50Hz CREE fading interrupt
	initTimerA(TIMER_A3, 50);
	TIMER_A3->CTL &= ~TIMER_A_CTL_IE; // Off for now, not in MODE_SETUP at startup
	NVIC_EnableIRQ(TA3_N_IRQn);
}

/**
 * Interrupt handlers
 */

void TA1_N_IRQHandler(void) {
	TIMER_A1->CTL &= ~TIMER_A_CTL_IFG;
	flags |= STATUS_REPORT_FLAG;
}

void TA2_N_IRQHandler(void) {
	TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;
	flags |= SEND_FLAG;
}

void TA3_N_IRQHandler(void) {
	TIMER_A3->CTL &= ~TIMER_A_CTL_IFG;
	static uint8_t tics = 0;
	switch (mode) {
	case MODE_SETUP:
		// Make pwm a sine wave
		break;
	case MODE_CONNECTION_DROPPED:
		if (tics == 10) pwm(prefs.brightness, CCR1);
		else if (tics == 20) {
			tics = 0;
			pwm(0, CCR1);
		}
		tics++;
		break;
	}
}

void EUSCIA1_IRQHandler(void) {
	uint8_t byte;
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_RXIFG) {
		byte = EUSCI_A1_SPI->RXBUF;
		if (!(flags & STATUS_READ_FLAG)) {
			nrfStatus = byte;
			if (nrfStatus & NRF_STATUS_TX_DS) flags &= ~PACKET_DROP_FLAG;
			if (nrfStatus & NRF_STATUS_MAX_RT) flags |= PACKET_DROP_FLAG;
			flags |= STATUS_READ_FLAG;
		}
		addToBuff(&miso, byte);
	}
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&mosi, &byte) == ERR_NO) EUSCI_A1_SPI->TXBUF = byte;
		else EUSCI_A1_SPI->IE &= ~EUSCI_A_IE_TXIE;
	}
}

void PORT5_IRQHandler (void) {
	if (P5->IFG & BIT1) {
		// Right button pressed
	}
	if (P5->IFG & BIT6) {
		flags |= GET_NRF_STATUS_FLAG;
	}

	P5->IFG = 0;
}









#else // Car











void main(void) {

}

#endif





















#ifdef TETSING2
#define NRF_SEND() EUSCI_A1_SPI->IE |= EUSCI_A_IE_TXIE; \
                   EUSCI_A2_SPI->IE |= EUSCI_A_IE_TXIE; \
	               while (EUSCI_A1_SPI->STATW & EUSCI_A_STATW_BUSY || EUSCI_A2_SPI->STATW & EUSCI_A_STATW_BUSY);

uint8_t nrfRXInt = 0;
uint8_t nrfTXInt = 0;
uint8_t rxStatusRead = 0;
uint8_t txStatusRead = 0;
uint8_t rxStatus = 0;
uint8_t txStatus = 0;
Buff mosiBuffTX = {0};
Buff misoBuffTX = {0};
Buff mosiBuffRX = {0};
Buff misoBuffRX = {0};

uint64_t test = 0;
void setupRX(void);
void setupTX(void);
void clearNRFRXFlags(void);
void clearNRFTXFlags(void);
void getLineHandler(char * line, error err) {
	if (err) logIS(errorToStr(err));
	else {
		char str[20];
		logIS(line);
		err = strToUInt64(line, &test);
		if (err) logIS(errorToStr(err));
		else {
			uInt64ToHexStr(test, str);
			logIS(str);
		}
	}
	if (line) free(line);
}
void commandHandler(uint8_t received) {
	getLine(&getLineHandler);
}

void main(void) {
	uint8_t data = 0;
	int i = 0;
	char str[19];
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog timer
	setCPUFreq(FREQ_12);
	configLogging(EUSCI_A0, 115200, 32);
	P1->SEL0 |= BIT2 | BIT3;                  // 2 = RX 3 = TX
	P1->SEL1 &= ~(BIT2 | BIT3);               // Set to primary function (01)
	startLogging(EUSCIA0_IRQn);

	enableCommands(&commandHandler);

	initBuff(&mosiBuffTX, 10);
	initBuff(&misoBuffTX, 10);
	initBuff(&mosiBuffRX, 10);
	initBuff(&misoBuffRX, 10);

	initTimerA(TIMER_A0, 60);
	/* Setup pin for PWM
	 * Needs to attach to OUT1 of TA0 (See page 147 of specific DS)
	 */
	P2->DIR |= BIT2;
	mapPin(PORT_2, PIN_2, PMAP_TA0CCR1A);
//	pwm(1, CCR1); // Duty cycle = 1 / 1000, use cap comp reg 1


	// Setup interrupt and CE pins
	P5->OUT &= ~BIT1;
	P5->DIR |= BIT1;
	P5->DIR &= ~BIT0;
	P5->IFG = 0;
	P5->IES |= BIT0;    // Interrupt fires on high to low transition
	P5->IE |= BIT0;
	NVIC_EnableIRQ(PORT5_IRQn);     // Register port 1 interrupts with NVIC
	P6->OUT &= ~BIT1; // Stay high for constant Standby II mode.
	P6->DIR |= BIT1;
	P6->DIR &= ~BIT0;
	P6->IFG = 0;
	P6->IES |= BIT0;    // Interrupt fires on high to low transition
	P6->IE |= BIT0;
	NVIC_EnableIRQ(PORT6_IRQn);     // Register port 1 interrupts with NVIC


	/* Configure SPI
	 */
	configSPIMaster(EUSCI_A1_SPI);
	configSPIMaster(EUSCI_A2_SPI);
	mapPin(PORT_2, PIN_3, PMAP_UCA1STE);
	mapPin(PORT_2, PIN_5, PMAP_UCA1CLK);
	mapPin(PORT_2, PIN_6, PMAP_UCA1SOMI);
	mapPin(PORT_2, PIN_7, PMAP_UCA1SIMO);
	mapPin(PORT_3, PIN_3, PMAP_UCA2STE);
	mapPin(PORT_3, PIN_5, PMAP_UCA2CLK);
	mapPin(PORT_3, PIN_6, PMAP_UCA2SOMI);
	mapPin(PORT_3, PIN_7, PMAP_UCA2SIMO);
	startSPI(EUSCI_A1_SPI);
	startSPI(EUSCI_A2_SPI);
	EUSCI_A1_SPI->IE |= EUSCI_A_IE_TXIE | EUSCI_A_IE_RXIE;
	EUSCI_A2_SPI->IE |= EUSCI_A_IE_TXIE | EUSCI_A_IE_RXIE;
	NVIC_EnableIRQ(EUSCIA1_IRQn);
	NVIC_EnableIRQ(EUSCIA2_IRQn);

	initTimerA(TIMER_A2, 5);
	TIMER_A2->CTL |= TIMER_A_CTL_IE;
//	TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE;
//	NVIC_EnableIRQ(TA1_0_IRQn);
	NVIC_EnableIRQ(TA2_N_IRQn);

	P1->DIR &= ~(BIT1 | BIT4);  // Buttons S1/S2 set to input
	P1->REN |= BIT1 | BIT4;     // Enable pullup/down resistors
	P1->OUT |= BIT1 | BIT4;     // Set to pullup mode
	P1->IFG = 0;                // Clear the interrupt Flag
	P1->IES |= BIT1 | BIT4;     // Interrupt fires on high to low transition
	P1->IE |= BIT1 | BIT4;      // Enable interrupt for buttons
	NVIC_EnableIRQ(PORT1_IRQn); // Register port 1 interrupts with NVIC

	setupRX();
	setupTX();
	// Setup RX/TX led
	P1->OUT &= ~BIT0;
	P1->DIR |= BIT0;


	TIMER_A1->CCR[0] = 0;                       // Stop timer for config
	TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK |   // Use SMCLK
			        TIMER_A_CTL_ID__1 |          // Input divider
	                TIMER_A_CTL_MC__UP;         // Count up to TA0CCR0
	TIMER_A1->EX0 = TIMER_A_EX0_IDEX__1;
	TIMER_A1->CCTL[0] = 0;                      // Defaults look good
	TIMER_A1->CCTL[1] = TIMER_A_CCTLN_OUTMOD_0; // Output bit, default 0
	TIMER_A1->CCTL[2] = TIMER_A_CCTLN_OUTMOD_0; // Output bit, default 0
	TIMER_A1->CCTL[3] = TIMER_A_CCTLN_OUTMOD_0; // Output bit, default 0
	TIMER_A1->CCTL[4] = TIMER_A_CCTLN_OUTMOD_0; // Output bit, default 0
	TIMER_A1->CTL |= TIMER_A_CTL_CLR;           // Reset the timer
	TIMER_A1->CCR[0] = 2;            // Start timer at 200Hz
	TIMER_A1->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;  // Set / Reset
	TIMER_A1->CCR[1] = 1;
	P2->DIR |= BIT0;
	mapPin(PORT_2, PIN_0, PMAP_TA1CCR1A);


	while(1) {
		for (; i < 0xFFFFF; i++);
		logIS("This is in the while loop lksadjflkasdj flkasjdflk");
		if (nrfRXInt) { // Received
			P1->OUT |= BIT0;
			logIS("                     Received");
			addToBuff(&mosiBuffRX, NRF_R_RX_PAYLOAD);
			addToBuff(&mosiBuffRX, NRF_WAIT_FOR_DATA);
			NRF_SEND();
			clearNRFRXFlags();
			nrfRXInt = 0;
			P1->OUT &= ~BIT0;
		}
		if (nrfTXInt) { // sent
			P1->OUT |= BIT0;
			logIS("Sent");
			clearNRFTXFlags();
			nrfTXInt = 0;
			P1->OUT &= ~BIT0;
		}
		if (getFromBuff(&misoBuffRX, &data) == ERR_NO) {
			logIS("                    RX Got: ");
			uInt8ToHexStr(data, str);
			printStringIS(str);
		}
		if (getFromBuff(&misoBuffTX, &data) == ERR_NO) {
			logIS("TX Got: ");
			uInt8ToHexStr(data, str);
			printStringIS(str);
		}
	}
}

void clearNRFRXFlags(void) {
	// Clear interrupt flags
	addToBuff(&mosiBuffRX, NRF_W_REGISTER(NRF_STATUS_ADDR));
	addToBuff(&mosiBuffRX, NRF_STATUS_RX_DR |
			               NRF_STATUS_TX_DS |
						   NRF_STATUS_MAX_RT |
						   NRF_STATUS_RX_P_NO);
	NRF_SEND();
}

void clearNRFTXFlags(void) {
	// Clear interrupt flags
	addToBuff(&mosiBuffTX, NRF_W_REGISTER(NRF_STATUS_ADDR));
	addToBuff(&mosiBuffTX, NRF_STATUS_RX_DR |
                           NRF_STATUS_TX_DS |
						   NRF_STATUS_MAX_RT |
						   NRF_STATUS_RX_P_NO);
	NRF_SEND();
}

void setupRX(void) {
	addToBuff(&mosiBuffRX, NRF_W_REGISTER(NRF_CONFIG_ADDR));
	addToBuff(&mosiBuffRX, NRF_CONFIG_TX_DS_IIE |
                           NRF_CONFIG_MAX_RT_IIE |
			               NRF_CONFIG_PWR_UP |
			               NRF_CONFIG_PRIM_RX);
	NRF_SEND();
	addToBuff(&mosiBuffRX, NRF_W_REGISTER(NRF_RX_PW_P0_ADDR));
	addToBuff(&mosiBuffRX, 1);
	NRF_SEND();
	addToBuff(&mosiBuffRX, NRF_W_REGISTER(NRF_RF_SETUP_ADDR));
	addToBuff(&mosiBuffRX, NRF_RF_SETUP_250KBPS |
						   NRF_RF_SETUP_RF_PWR_0);
	NRF_SEND();
	addToBuff(&mosiBuffRX, NRF_W_REGISTER(NRF_FEATURE_ADDR));
	addToBuff(&mosiBuffRX, NRF_FEATURE_EN_DYN_ACK);
	NRF_SEND();
	P5->OUT |= BIT1; // Start receiver
}

void setupTX(void) {
	addToBuff(&mosiBuffTX, NRF_W_REGISTER(NRF_CONFIG_ADDR));
	addToBuff(&mosiBuffTX, NRF_CONFIG_RX_DR_IIE |
			               NRF_CONFIG_PWR_UP |
			               NRF_CONFIG_PRIM_TX);
	NRF_SEND();
	addToBuff(&mosiBuffTX, NRF_W_REGISTER(NRF_RF_SETUP_ADDR));
	addToBuff(&mosiBuffTX, NRF_RF_SETUP_250KBPS |
						   NRF_RF_SETUP_RF_PWR_0);
	NRF_SEND();
	addToBuff(&mosiBuffTX, NRF_W_REGISTER(NRF_FEATURE_ADDR));
	addToBuff(&mosiBuffTX, NRF_FEATURE_EN_DYN_ACK);
	NRF_SEND();
	P6->OUT |= BIT1;
}


/*
 *
	// Setup receiver

	addToBuff(&mosiBuffRX, NRF_R_REGISTER(NRF_RPD_ADDR));
	addToBuff(&mosiBuffRX, NRF_WAIT_FOR_DATA);
	NRF_SEND();
	addToBuff(&mosiBuffRX, NRF_W_REGISTER(NRF_CONFIG_ADDR));
	addToBuff(&mosiBuffRX, NRF_CONFIG_TX_DS_IIE |
                           NRF_CONFIG_MAX_RT_IIE |
			               NRF_CONFIG_PWR_UP |
			               NRF_CONFIG_PRIM_RX);
	// Setup transmitter
	addToBuff(&mosiBuffTX, NRF_W_REGISTER(NRF_CONFIG_ADDR));
	addToBuff(&mosiBuffTX, NRF_CONFIG_RX_DR_IIE |
                           NRF_CONFIG_MAX_RT_IIE |
			               NRF_CONFIG_PWR_UP |
			               NRF_CONFIG_PRIM_TX);
	logIS("Writing CONFIG");
	NRF_SEND();
	logIS("Flushing stuff");
	// Setup transmitter
	addToBuff(&mosiBuffRX, NRF_FLUSH_RX);
	addToBuff(&mosiBuffTX, NRF_FLUSH_TX);
	NRF_SEND();
	// Setup transmitter
	NRF_SEND();
	logIS("Enabling NO_ACK stuff");
	addToBuff(&mosiBuffTX, NRF_W_REGISTER(NRF_FEATURE_ADDR));
	addToBuff(&mosiBuffTX, NRF_FEATURE_EN_DYN_ACK);
	NRF_SEND();
	logIS("Enabling cont wave");
	addToBuff(&mosiBuffTX, NRF_W_REGISTER(NRF_RF_SETUP_ADDR));
	addToBuff(&mosiBuffTX, NRF_RF_SETUP_CONT_WAVE |
			               NRF_RF_SETUP_PLL_LOCK |
						   NRF_RF_SETUP_RF_DR_HIGH |
						   NRF_RF_SETUP_RF_PWR_0);
	NRF_SEND();
	logIS("Clearing flags");
	clearNRFRXFlags();
	clearNRFTXFlags();

	P5->OUT |= BIT1;
	P6->OUT |= BIT1;
 */

void MemManage_Handler(void) {
	while(1);
}

void TA2_N_IRQHandler(void) {
	TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;
//	logIS("Testing, testing, 1, 2, 3, 0xA, 0xB, 0xC jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj");
//	logIS("Sending 'A'");

//	addToBuff(&mosiBuffTX, NRF_W_TX_PAYLOAD_NO_ACK);
//	addToBuff(&mosiBuffTX, 'A');
//	NRF_SEND();

//	addToBuff(&mosiBuffRX, NRF_R_REGISTER(NRF_RPD_ADDR));
//	addToBuff(&mosiBuffRX, NRF_WAIT_FOR_DATA);
//	NRF_SEND();


}

void EUSCIA1_IRQHandler(void) {
	uint8_t byte;
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_RXIFG) {
		byte = EUSCI_A1_SPI->RXBUF;
		if (!txStatusRead) {
			txStatus = byte;
			txStatusRead = 1;
		}
		addToBuff(&misoBuffTX, byte);
	}
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&mosiBuffTX, &byte) == ERR_NO) EUSCI_A1_SPI->TXBUF = byte;
		else {
			EUSCI_A1_SPI->IE &= ~EUSCI_A_IE_TXIE;
			txStatusRead = 0;
		}
	}
}

void EUSCIA2_IRQHandler(void) {
	uint8_t byte;
	if (EUSCI_A2_SPI->IFG & EUSCI_A_IFG_RXIFG) {
		byte = EUSCI_A2_SPI->RXBUF;
		if (!rxStatusRead) {
			rxStatus = byte;
			rxStatusRead = 1;
		}
		addToBuff(&misoBuffRX, byte);
	}
	if (EUSCI_A2_SPI->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&mosiBuffRX, &byte) == ERR_NO) EUSCI_A2_SPI->TXBUF = byte;
		else {
			EUSCI_A2_SPI->IE &= ~EUSCI_A_IE_TXIE;
			rxStatusRead = 0;
		}
	}
}

void PORT1_IRQHandler (void) {
	logIS("Wow, is this what the inside of an interrupt handler looks like?");
	P1->IFG = 0;
}

void PORT5_IRQHandler (void) {
	nrfRXInt = 1;
	P5->IFG = 0;
}

void PORT6_IRQHandler (void) {
	nrfTXInt = 1;
	P6->IFG = 0;
}
#endif




































#ifdef TESTING
#define PULSE_WIDTH (100)
#define DUTY_CYCLE (0.01)
#define RTC_KEY ((uint16_t) 0xA500)

Buff spiTXBuff = {0};
Buff spiRXBuff = {0};
Buff slaveTXBuff = {0};
Buff slaveRXBuff = {0};
Buff uartTXBuff = {0};
Buff uartRXBuff = {0};
uint8_t send = 0;
uint16_t time = 0;

void inline sendMaster(uint8_t byte);
void inline sendSlave(uint8_t byte);
void configureButtons(void);
void sendCommandSPI(uint8_t command);
void inline printByte(uint8_t byte);
void printBytes(uint8_t * array, uint32_t length);
void log(char * string);
void printNewline(void);


void main(void) {
	uint8_t resp = 0;
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;                      // Stop watchdog timer

	initBuff(&spiTXBuff, 6);
	initBuff(&spiRXBuff, 6);
	initBuff(&slaveTXBuff, 6);
	initBuff(&slaveRXBuff, 6);
	initBuff(&uartTXBuff, 32);
	initBuff(&uartRXBuff, 6);

	/* Setup RTC
	 */
	RTC_C->CTL0 = RTC_KEY;               // Unlock register
	RTC_C->CTL0 = RTC_C_CTL0_RDYIE |     // Interrupt every second to update time
	              RTC_KEY;               // Stay unlocked
	RTC_C->CTL13 = RTC_C_CTL13_RDY;      // Calendar mode (0 is reserved)
	RTC_C->CTL0 &= ~RTC_C_CTL0_KEY_MASK; // Relock by clearing key
//	NVIC_EnableIRQ(RTC_C_IRQn);

	/* Setup UART
	 */
	enableUART(CS_CTL0_DCORSEL_1, 115200);
	enableInterruptsUART(NULL, EUSCI_A_IE_RXIE);
	log("           Test!!!!");
	printNewline();

	/* For SPI
	 */

	/* Setup led
	 */
	P1->SEL1 &= ~BIT0;
	P1->SEL0 &= ~BIT0;
	P1->DIR |= BIT0;
	P1->OUT &= ~BIT0;

	TIMER_A1->CCR[0] = 0;                          // Stop timer for config
	TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK |      // Use SMCLK
	                TIMER_A_CTL_ID__8 |            // Input divider
	                TIMER_A_CTL_MC__UP;            // Count up to TA1CCR0
	TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE;
	TIMER_A1->CTL |= TIMER_A_CTL_CLR;              // Reset the timer
	TIMER_A1->CCR[0] = 60000;
	NVIC_EnableIRQ(TA1_0_IRQn);

	/* Setup input buttons
	 */
	configureButtons();

	/* Setup eUSCI_A1 for SPI
	 */
	EUSCI_A1_SPI->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI_A1 in reset
	EUSCI_A1_SPI->CTLW0 = EUSCI_A_CTLW0_CKPH |   // phase = 1 to shift on falling edge
	                      EUSCI_A_CTLW0_MSB |    // MSB first
	                      EUSCI_A_CTLW0_MST |    // Master
	                      EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                      EUSCI_A_CTLW0_SYNC |   // SPI mode
	                      EUSCI_A_CTLW0_SSEL__SMCLK |
	                      EUSCI_A_CTLW0_STEM |   // STE used for slave
	                      EUSCI_A_CTLW0_SWRST;   // STE used for slave
	EUSCI_A1_SPI->BRW = 8;                       // Divide by enough to safely use 48MHz
	EUSCI_A1_SPI->STATW = 0;                     // Defaults

	/* Setup pin for SPI
	 */
	P2->SEL1 &= ~(BIT3 | BIT5 | BIT6 | BIT7);
	P2->SEL0 |= BIT3 | BIT5 | BIT6 | BIT7;
	PMAP->KEYID = PMAP_KEYID_VAL;                // Unlock PMAP controller
	PMAP->CTL |= PMAP_CTL_PRECFG;                // Allow it to be reconfigured later
	P2MAP->PMAP_REGISTER3 = PMAP_UCA1STE;
	P2MAP->PMAP_REGISTER5 = PMAP_UCA1CLK;
	P2MAP->PMAP_REGISTER6 = PMAP_UCA1SOMI;
	P2MAP->PMAP_REGISTER7 = PMAP_UCA1SIMO;
	PMAP->KEYID = 0;                             // Lock PMAP controller
	EUSCI_A1_SPI->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Release for operation
	EUSCI_A1_SPI->IE = EUSCI_A_IE_TXIE |         // TX interrupt
	                   EUSCI_A_IE_RXIE;          // RX interrupt
	NVIC_EnableIRQ(EUSCIA1_IRQn);

	/* Setup extra pins to NRF
	 */
	P3->DIR |= BIT5 | BIT7;
	P3->OUT &= ~(BIT7);
	P3->OUT |= BIT5;

	P6->DIR &= ~BIT6;
	P6->REN |= BIT6;            // Enable pullup/down resistors
	P6->OUT |= BIT6;            // Set to pullup mode
	P6->IFG = 0;                // Clear the interrupt Flag
	P6->IES |= BIT6;            // Interrupt fires on high to low transition
	P6->IE |= BIT6;             // Enable interrupt for buttons
	NVIC_EnableIRQ(PORT6_IRQn); // Register port 1 interrupts with NVIC










	/* Setup eUSCI_A1 for SPI (test slave)
	 */
	EUSCI_A2_SPI->CTLW0 |= EUSCI_A_CTLW0_SWRST;  // Put eUSCI_A2 in reset
	EUSCI_A2_SPI->CTLW0 = EUSCI_A_CTLW0_CKPH |
	                      EUSCI_A_CTLW0_MSB |    // MSB first
	                      EUSCI_A_CTLW0_MODE_2 | // 4-bit, slave active on STE low
	                      EUSCI_A_CTLW0_SYNC |   // SPI mode
	                      EUSCI_A_CTLW0_SWRST;   // STE used for slave
	EUSCI_A2_SPI->STATW = 0;                     // Defaults

	/* Setup pin for SPI
	 */
	P3->SEL1 &= ~(BIT3 | BIT5 | BIT6 | BIT7);
	P3->SEL0 |= BIT3 | BIT5 | BIT6 | BIT7;
	PMAP->KEYID = PMAP_KEYID_VAL;                // Unlock PMAP controller
	PMAP->CTL |= PMAP_CTL_PRECFG;                // Allow it to be reconfigured later
	P3MAP->PMAP_REGISTER3 = PMAP_UCA2STE;
	P3MAP->PMAP_REGISTER5 = PMAP_UCA2CLK;
	P3MAP->PMAP_REGISTER6 = PMAP_UCA2SOMI;
	P3MAP->PMAP_REGISTER7 = PMAP_UCA2SIMO;
	PMAP->KEYID = 0;                             // Lock PMAP controller
	EUSCI_A2_SPI->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Release for operation
	EUSCI_A2_SPI->IE = EUSCI_A_IE_TXIE |        // TX interrupt
	                   EUSCI_A_IE_RXIE;         // RX interrupt
	EUSCI_A2_SPI->IFG = 0;
	NVIC_EnableIRQ(EUSCIA2_IRQn);
	EUSCI_A1_SPI->TXBUF = 0xBEEF;











	/* Setup the PWM output on P2.6 for the LCD. The ordering is important here,
	 * we stop the timer first, then setup the general control register,
	 * followed by CCTL0 for the pulse rate, then the CCTLn registers to set the
	 * duty cycle. After this we reset the timer and start it counting again by
	 * setting CCR0 to a non-zero value.
	 */
	TIMER_A0->CCR[0] = 0;                          // Stop timer for config
	TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |      // Use SMCLK
	                TIMER_A_CTL_ID__1 |            // Input divider
	                TIMER_A_CTL_MC__UP;            // Count up to TA0CCR0
	TIMER_A0->CCTL[0] = 0;                         // Defaults look good
	TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;    // Set/Reset
	TIMER_A0->CCR[1] = (uint16_t) (DUTY_CYCLE * PULSE_WIDTH);

	/* Setup pin for PWM
	 * Needs to attach to OUT3 of TA0 (See page 147 of specific DS)
	 */
	P2->DIR |= BIT2;
	P2->SEL1 &= ~BIT2;
	P2->SEL0 |= BIT2;
	PMAP->KEYID = PMAP_KEYID_VAL;                  // Unlock PMAP controller
	PMAP->CTL |= PMAP_CTL_PRECFG;                  // Allow it to be reconfigured later
	P2MAP->PMAP_REGISTER2 = PMAP_TA0CCR1A;         // Tie to OUT1 from TA0
	PMAP->KEYID = 0;                               // Lock PMAP controller

	TIMER_A0->CTL |= TIMER_A_CTL_CLR;              // Reset the timer
	TIMER_A0->CCR[0] = PULSE_WIDTH;                // Start timer at 200Hz
	__enable_interrupts();

	while(1) {
		time++;
		if (send) {
			sendMaster('F');
			send = 0;
		}
		if (getFromBuff(&uartRXBuff, &resp) == BUFF_NO_ERR) {
			log("UART got: ");
			printByte(resp);
			printNewline();
		}
		if (getFromBuff(&spiRXBuff, &resp) == BUFF_NO_ERR) {
			log("Master got: ");
			printByte(resp);
			printNewline();
		}
		if (getFromBuff(&slaveRXBuff, &resp) == BUFF_NO_ERR) {
			log("Slave got: ");
			printByte(resp);
			printNewline();
			sendSlave(resp);
		}
	}
}





void sendCommandSPI(uint8_t command) {
	if (!(EUSCI_A1_SPI->IFG & EUSCI_A_IFG_TXIFG)) {
		while (addToBuff(&spiTXBuff, command) != BUFF_NO_ERR); // Keep trying until it works
	}
	else EUSCI_A1_SPI->TXBUF = command;
}

void inline sendMaster(uint8_t byte) {
	log("Master sending: ");
	printByte(byte);
	printNewline();
	addToBuff(&spiTXBuff, byte);
	addToBuff(&spiTXBuff, byte - 3);
	EUSCI_A1_SPI->IE |= EUSCI_A_IE_TXIE;
}

void inline sendSlave(uint8_t byte) {
	log("Slave sending: ");
	printByte(byte);
	printNewline();
	addToBuff(&slaveTXBuff, byte);
	EUSCI_A2_SPI->IE |= EUSCI_A_IE_TXIE;
}






void inline printByte(uint8_t byte) {
	while(addToBuff(&uartTXBuff, byte) != BUFF_NO_ERR);
	EUSCI_A0->IE |= EUSCI_A_IE_TXIE;
}

void printBytes(uint8_t * array, uint32_t length) {
	while (length--) printByte(*(array++)); // Increment up array, printing byte by byte until length is hit
}

void log(char * string) {
	char nowStr[12];
	char * nowPtr = nowStr;
//	itoa((uint32_t) RTC_C->TIM0, nowStr);
	itoa((uint32_t) time, nowStr);
		while (*nowPtr) printByte(*(nowPtr++));
	printByte(':');
	printByte(' ');
	while (*string) printByte(*(string++)); // Increment up string, printing char by char until '\0' is hit
}

void printNewline(void) {
	printByte('\n'); // Transmit a line break
	printByte('\r');
}






/* Interrupt handlers
 */

void TA1Handler(void) {
	P1OUT ^= BIT0;
	TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
	send = 1;
}

void Port1Handler(void) {
	P1OUT ^= BIT0;
	if (P1->IFG & BIT1) {
		log("Hey!");
		printNewline();
	}
	if (P1->IFG & BIT4) {
		log("Hey!");
		printNewline();
	}
	P1->IFG = 0;
}

void Port6Handler(void) {
	// NRF received somthin'
	P6->IFG = 0;
}

void UARTHandler(void) {
	uint8_t data;
	if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) {
		addToBuff(&uartRXBuff, EUSCI_A0->RXBUF);
	}
	if (EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&uartTXBuff, &data) == BUFF_NO_ERR) EUSCI_A0->TXBUF = data;
		else EUSCI_A0->IE &= ~EUSCI_A_IE_TXIE;
	}
}

void SPIHandler(void) {
	nrfCmd command = 0;
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_RXIFG) {
		addToBuff(&spiRXBuff, EUSCI_A1_SPI->RXBUF);
	}
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&spiTXBuff, &command) == BUFF_NO_ERR) EUSCI_A1_SPI->TXBUF = command;
		else EUSCI_A1_SPI->IE &= ~EUSCI_A_IE_TXIE;
	}
}

void SPISlaveHandler(void) {
	uint8_t data = 0;
	if (EUSCI_A2_SPI->IFG & EUSCI_A_IFG_RXIFG) {
		addToBuff(&slaveRXBuff, EUSCI_A2_SPI->RXBUF - 1);
	}
	if (EUSCI_A2_SPI->IFG & EUSCI_A_IFG_TXIFG) {
		if (getFromBuff(&slaveTXBuff, &data) == BUFF_NO_ERR) EUSCI_A2_SPI->TXBUF = data;
		else EUSCI_A2_SPI->IE &= ~EUSCI_A_IE_TXIE;
	}
}

void RTCHandler(void) {
	P1OUT ^= BIT0;
//	RTC_C->CTL0 = RTC_KEY;               // Unlock register
//	RTC_C->CTL0 |= RTC_C_CTL0_RDYIE;     // Interrupt every second to update time
	time = RTC_C->TIM0;
//	RTC_C->CTL0 &= ~RTC_C_CTL0_RDYIFG;
//	RTC_C->CTL0 &= ~RTC_C_CTL0_KEY_MASK; // Relock by clearing key
}










void configureButtons(void) {
	P1->DIR &= ~(BIT1 | BIT4);  // Buttons S1/S2 set to input
	P1->REN |= BIT1 | BIT4;     // Enable pullup/down resistors
	P1->OUT |= BIT1 | BIT4;     // Set to pullup mode
	P1->IFG = 0;                // Clear the interrupt Flag
	P1->IES |= BIT1 | BIT4;     // Interrupt fires on high to low transition
	P1->IE |= BIT1 | BIT4;      // Enable interrupt for buttons
	NVIC_EnableIRQ(PORT1_IRQn); // Register port 1 interrupts with NVIC
}
#endif
