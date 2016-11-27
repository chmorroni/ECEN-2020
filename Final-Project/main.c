#include <stdint.h>
#include <stdarg.h>
#include "msp.h"
#include "core_cm4.h"
#include "Libs/nrf.h"
#include "Libs/async.h"
#include "Libs/buffer.h"
#include "Libs/serial.h"
#include "Libs/error.h"
#include "Libs/helpers.h"
#include "Libs/timers.h"
#include "Libs/logging.h"
#include "Libs/portMapping.h"

#define TRANSMITTER

/**
 * Global macros
 */
#define DIR_OFS      (0)
#define SPEED_OFS    (2)
#define CMD_OFS      SPEED_OFS
#define DIR_MASK     (0x03)
#define SPEED_MASK   (0xFC)
#define CMD_MASK     SPEED_MASK
#define CMD_ENCODING (0x03)
#define MOSI_SIZE    (4)
#define MISO_SIZE    (4)
#define PI           (3.1416)

/**
 * Global types
 */
typedef enum {
	CMD_NOP,           // No op. Could be used to test for connection.

	CMD_LIGHTS_EN,     // Turn on all lights
	CMD_LIGHTS_DI,     // Turn off all lights
	CMD_LIGHTS_BLINK,  // Blink lights

	CMD_MOVE_EN,       // Enable movement
	CMD_MOVE_DS,       // Disable movement

	CMD_MODE_BREAK_EN, // Enable breaking on stop
	CMD_MODE_BREAK_DS, // Disable breaking on stop
	CMD_MODE_SLOW_EN,  // Enable max top speed
	CMD_MODE_SLOW_DS,  // Disable max top speed
	CMD_MODE_SAFE_EN,  // Enable sensors
	CMD_MODE_SAFE_DS,  // Disable sensors
} command;

typedef enum {
	DIR_STRAIT,
	DIR_RIGHT,
	DIR_LEFT
} direction;

typedef struct {
	REG(speed, 6); // 6 bits wide, -32 to 31
	direction dir;
} state;

/**
 * Global variables
 */
char * commandDescriptions[] = {
		"No-op",

		"Enable lights",
		"Disable lights",
		"Blink lights",

		"Enable movement",
		"Disable movement",

		"Enable breaking on 0 speed",
		"Disable breaking on 0 speed",
		"Enable speed governor",
		"Disable speed governor",
		"Enable safe mode",
		"Disable safe mode",
};
char * directionDescriptions[] = {
		"Strait",
		"Right",
		"Left",
};
// Sine lookup table (0 to PI / 2)
static float sines[] = {0.0000, 0.0628, 0.1253, 0.1874, 0.2487, 0.3090, 0.3681,
                        0.4258, 0.4818, 0.5358, 0.5878, 0.6374, 0.6845, 0.7290,
                        0.7705, 0.8090, 0.8443, 0.8763, 0.9048, 0.9298, 0.9511,
                        0.9686, 0.9823, 0.9921, 0.9980, 1.0000};


static union {
	uint8_t reg;
	struct { // Anonymous struct for accessing bit fields in the status register.
		FLAG(txFull);
		UREG(rxPipeNum, 3);
		FLAG(maxRT);
		FLAG(txDS);
		FLAG(rxDR);
		FLAG(reserved);
	};
} nrfStatus = {.reg = 0};
static Buff mosi = {0};
static Buff miso = {0};

/**
 * Global function prototypes
 */

/**
 * Helper functions
 */
inline float sin(uint8_t theta);
uint8_t encodeState(state stateToEncode);
uint8_t encodeCommand(command commandToEncode);
void decodePacket(uint8_t packet, command * commandContainer, state * stateContainer);

/**
 * NRF functions
 */
inline void nrfEnCE(void);
inline void nrfDsCE(void);
inline void nrfWaitForSPI(void);
inline void nrfSendSPI(void);
inline void nrfUpdateStatus(void);
inline void nrfFlushTXFIFO(void);
inline void nrfFlushRXFIFO(void);
inline void nrfRReg(uint8_t reg);
inline void nrfWReg(uint8_t reg, uint8_t payload);
inline void nrfReadRX(void);
inline void nrfTXState(state stateToTX);
inline void nrfTXCommand(command commandToTX);
inline void nrfClearStatus(void);





#ifdef TRANSMITTER

/**
 * Main code for transmitter
 */

/**
 *  Type definitions
 */
typedef enum {
	MODE_SETUP,              // Changing settings
	MODE_NORMAL,             // Controlling car
	MODE_DBUG,               // Controlling car
	MODE_RANGE_TEST,         // Expecting packet loss, displaying with LED
	MODE_CONNECTION_DROPPED, // Lost connection unexpectedly
	MODE_DISABLED            // Changing settings
} opMode;

/**
 *  Global vars
 */
static struct {
	FLAG(statusRead);
	FLAG(packetDropped);
	FLAG(sendCommandDbug); // Either or in dbug mode. Whether to send dictated by packets to send.
	FLAG(sendState);
	FLAG(sendCommand);
	FLAG(expectingRX);
	FLAG(log);
} flags = {0};
static uint32_t packetsSent = 0;
static uint32_t droppedPackets = 0;
static state nextState;
static command nextCommand;
static state nextStateDbug;
static command nextCommandDbug;
static uint32_t packetsToSend = 0;
static struct {
	float brightness;
	FLAG(breakMode);
	FLAG(fastMode);
	FLAG(safeMode);
} prefs = {0.1, 0, 0, 0};
static opMode mode = MODE_NORMAL;

/**
 * Callback functions
 */
void endRangeTestConfirm(char received);
void commandHandler(uint8_t received);

/**
 * Setup functions
 */
void setupLogging(void);
void setupTimers(void);
void setupInputs(void);
void setupNRF(void);
void setupLED(void);

/**
 * Helper functions
 */
inline void requestMode(opMode from, opMode to);
inline void forceMode(opMode to);
void printHelp(void);
void printInvalidCommand(uint8_t received);
inline void pauseStatusReports(void);
inline void resumeStatusReports(void);
void transmit(void);
void updateLED(opMode ledMode);


void main(void) { // Responsible for setup and user IO.
	char str[21]; // Enough for 2^64 + '\0' as well as 0xFFFFFF... + '\0'
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog timer
	setCPUFreq(FREQ_24);
	initAsync();
	setupLogging();
	printNewline();
	setupTimers();
	setupInputs();
	log("Setting up nrf");
	setupNRF();
	setupLED();
	log("Setup complete");

	__enable_interrupts();

	while(1) {
		updateLED(mode);
		switch (mode) {
		case MODE_SETUP:
			pauseStatusReports();
			while (mode == MODE_SETUP) {
				// TODO ...
			}
			break;
		case MODE_NORMAL:
			resumeStatusReports();
			while (mode == MODE_NORMAL) {

				if (flags.log) {
					log("Status Report:");
					pauseCommands();
					printString("Packets sent:    ");
					uIntToStr(packetsSent, str, 0);
					printString(str);
					printNewline();
					printString("Dropped packets: ");
					uIntToStr(droppedPackets, str, 0);
					printString(str);
					printNewline();
					printString("NRF status:      ");
					uInt8ToHexStr(nrfStatus.reg, str);
					printString(str);
					printNewline();
					printString("X:               ");
					uInt32ToHexStr(ADC14->MEM[0], str);
					printString(str);
					printNewline();
					printString("Y:               ");
					uInt32ToHexStr(ADC14->MEM[1], str);
					printString(str);
					printNewline();
					printNewline();
					packetsSent = 0;
					droppedPackets = 0;
					flags.log = 0;
					readyForCommands();
				}

			}
			break;
		case MODE_DBUG:
			resumeStatusReports();
			while (mode == MODE_DBUG) {
				if (flags.log) {
					log("Debugging status Report:");
					pauseCommands();
					printString("Packets sent:    ");
					uIntToStr(packetsSent, str, 0);
					printString(str);
					printNewline();
					printString("Packets left:    ");
					uIntToStr(packetsToSend, str, 0);
					printString(str);
					printNewline();
					printString("Dropped packets: ");
					uIntToStr(droppedPackets, str, 0);
					printString(str);
					printNewline();
					printString("NRF status:      ");
					uInt8ToHexStr(nrfStatus.reg, str);
					printString(str);
					printNewline();
					printNewline();
					packetsSent = 0;
					droppedPackets = 0;
					flags.log = 0;
				}
			}
			packetsToSend = 0;
			break;
		case MODE_RANGE_TEST:
			pauseStatusReports();
			while (mode == MODE_RANGE_TEST) {
				updateLED(mode);
			}
			break;
		case MODE_CONNECTION_DROPPED:
			pauseStatusReports();
			log("ERROR: Packet dropped.");
			log("Looking for transmitter...");
			while (mode == MODE_CONNECTION_DROPPED); // Handled by transmit function
			if (!flags.packetDropped) log("Receiver found. Resuming normal operation");
			break;
		case MODE_DISABLED:
			pauseStatusReports();
			TIMER_A2->CTL &= ~TIMER_A_CTL_IE; // Stop transmit interrupt
			nrfDsCE(); // Disable transmitter
			log("Transmitter disabled. You will not be able to do anything before re-enabling with the 'e' command.");
			while (mode == MODE_DISABLED); // Do nothing
			nrfEnCE(); // Enable transmitter
			TIMER_A2->CTL |= TIMER_A_CTL_IE; // Start transmit interrupt
			log("Transmitter operation restored.");
			break;
		}
	}
}

/**
 * Callback functions
 */

void endRangeTestConfirm(char received) {
	if (received == 'y' || received == 'Y') {
		printWrap("Range Test Mode deactivated. Resuming normal transmission.", 0, 0, 0);
		forceMode(MODE_NORMAL);
	}
	else {
		printWrap("Continuing in Range Test Mode.", 0, 0, 0);
		forceMode(MODE_RANGE_TEST);
	}
	readyForCommands();
}

void commandOptionHandler(uint32_t option) {
	nextCommandDbug = (command) option;
	flags.sendCommandDbug = 1;
	packetsToSend = 1;
	forceMode(MODE_DBUG);
	transmit();
	readyForCommands();
}

void speedHandler(int64_t number) {
	nextStateDbug.speed = number;
	flags.sendCommandDbug = 0;
	forceMode(MODE_DBUG);
	readyForCommands();
}

void directionOptionHandler(uint32_t option) {
	nextStateDbug.dir = (direction) option;
	printWrap("Choose a speed from -32 to 31.", 0, 0, 0);
	getNumberInRange(&speedHandler, -32, 31);
}

void packetsToSendHandler(int64_t number) {
	packetsToSend = number;
	printWrap("Choose a direction", 0, 0, 0);
	getOption(&directionOptionHandler, sizeof directionDescriptions / sizeof (char *), directionDescriptions);

}

void commandOrState(char received) {
	switch (received) {
	case 'c':
		printWrap("Choose a command", 0, 0, 0);
		getOption(&commandOptionHandler, sizeof commandDescriptions / sizeof (char *), commandDescriptions);
		break;
	case 's':
		printWrap("Choose a number of packets to send", 0, 0, 0);
		getNumberInRange(&packetsToSendHandler, 0, INT64_MAX);
		break;
	default:
		printWrap("No an option. Send command ('c') or state ('s')?", 0, 0, 0);
		getChar(&commandOrState);
	}
}

void commandHandler(uint8_t received) {
	if (mode == MODE_DISABLED && received != 'e') {
		log("Enable the transmitter with the 'e' command before proceeding.");
		return;
	}
	switch (received) {
	case 'd':
		forceMode(MODE_DISABLED);
		break;
	case 'e':
		forceMode(MODE_NORMAL);
		break;
	case 'i':
		pauseCommands();
		if (TIMER_A1->CTL & TIMER_A_CTL_IE) {
			pauseStatusReports();
			printWrap("Status reports turned off", 0, 0, 0);
		}
		else {
			resumeStatusReports();
			printWrap("Status reports turned on", 0, 0, 0);
		}
		readyForCommands();
		break;
	case 'h':
		pauseCommands();
		printHelp();
		readyForCommands();
		break;
	case 'p':
		// Preferences
		break;
	case 'r':
		pauseCommands();
		if (mode != MODE_RANGE_TEST) {
			printWrap("Range Test Mode activated. The LED will glow green when a transmitted package is acknowledged and red when it is not.", 0, 0, 0);
			forceMode(MODE_RANGE_TEST);
			readyForCommands();
		}
		else {
			if (flags.packetDropped) {
				printWrap("Not in range of car. Are you sure you want to deactivate Range Test Mode? (y|n)", 0, 0, 0);
				getChar(&endRangeTestConfirm);
			}
			else endRangeTestConfirm('Y');
		}
		break;
	case 's':
		pauseCommands();
		printWrap("Send command ('c') or state ('s')?", 0, 0, 0);
		getChar(&commandOrState);
		// Get speed
		// Get turn direction
		// Get number to send
		// Send command
		break;
	default:
		pauseCommands();
		printInvalidCommand(received);
		readyForCommands();
		break;
	}
}



/**
 * Setup functions
 */
void setupLogging(void) {
	configLogging(EUSCI_A0, 115200);
	P1->SEL0 |= BIT2 | BIT3;                    // 2 = RX 3 = TX
	P1->SEL1 &= ~(BIT2 | BIT3);                 // Set to primary function (01)
	startLogging(EUSCIA0_IRQn);
	enableCommands(&commandHandler);
}

void setupTimers(void) {
	// 1 Hz logging interrupt
	initTimerA(TIMER_A1, 1);
	TIMER_A1->CTL |= TIMER_A_CTL_IE;
	NVIC_EnableIRQ(TA1_N_IRQn);

	// State packet sending interrupt
	initTimerA(TIMER_A2, 50);
	TIMER_A2->CTL |= TIMER_A_CTL_IE;
	NVIC_EnableIRQ(TA2_N_IRQn);
}

void setupInputs(void) {
	// MSP buttons
	P1->DIR &= ~(BIT1 | BIT4);  // Buttons S1/S2 set to input
	P1->REN |= BIT1 | BIT4;     // Enable pullup/down resistors
	P1->OUT |= BIT1 | BIT4;     // Set to pullup mode
	P1->IFG = 0;                // Clear the interrupt Flag
	P1->IES |= BIT1 | BIT4;     // Interrupt fires on high to low transition
	P1->IE |= BIT1 | BIT4;      // Enable interrupt for buttons
	NVIC_EnableIRQ(PORT1_IRQn); // Register port 1 interrupts with NVIC

	// Left booster pack button
	P3->DIR &= ~BIT5;  // Buttons S1/S2 set to input
	P3->REN |= BIT5;     // Enable pullup/down resistors
	P3->OUT |= BIT5;     // Set to pullup mode
	P3->IFG = 0;                // Clear the interrupt Flag
	P3->IES |= BIT5;     // Interrupt fires on high to low transition
	P3->IE |= BIT5;      // Enable interrupt for buttons
	NVIC_EnableIRQ(PORT3_IRQn); // Register port 1 interrupts with NVIC

	// Right booster pack button
	P5->DIR &= ~BIT1;  // Buttons S1/S2 set to input
	P5->REN |= BIT1;     // Enable pullup/down resistors
	P5->OUT |= BIT1;     // Set to pullup mode
	P5->IFG = 0;                // Clear the interrupt Flag
	P5->IES |= BIT1;     // Interrupt fires on high to low transition
	P5->IE |= BIT1;      // Enable interrupt for buttons
	NVIC_EnableIRQ(PORT5_IRQn); // Register port 1 interrupts with NVIC

	// Joystick inputs
	// Setup reference generator
	P4->DIR &= ~BIT4;                          // X
	P4->SEL0 |= BIT4;                          // Select A9 mode
	P4->SEL1 |= BIT4;
	P6->DIR &= ~BIT0;                          // Y
	P6->SEL0 |= BIT0;                          // Select A15 mode
	P6->SEL1 |= BIT0;
	ADC14->CTL0 &= ~ADC14_CTL0_ENC;            // Allow changes
	ADC14->CTL0 = ADC14_CTL0_PDIV__1 | // predivider = 1
	              ADC14_CTL0_SHS_0 |       // Sample and hold select pulled from SC bit
	              ADC14_CTL0_SHP |         // Pulse mode
	              ADC14_CTL0_DIV__1 |      // divider = 1
	              ADC14_CTL0_SSEL__SMCLK | // SMCLK used for ADC14 clk
	              ADC14_CTL0_CONSEQ_3 |    // Repeat sequence of channels
	              ADC14_CTL0_SHT0__96 |    // 96 clocks per sample
	              ADC14_CTL0_SHT1__96 |    // 96 clocks per sample
	              ADC14_CTL0_MSC |         // Conversions do not require SHI after first one
	              ADC14_CTL0_ON;           // On

	ADC14->CTL1 = ADC14_CTL1_RES__8BIT |   // 8 bit resolution (For 5 bit width speed)
	              ADC14_CTL1_DF;
	ADC14->LO0 = 0x80;
	ADC14->HI0 = 0x7F;

	ADC14->MCTL[0] = ADC14_MCTLN_INCH_15;  // Map joyPos.x to MEM[0]
	ADC14->MCTL[1] = ADC14_MCTLN_INCH_9 |  // Map joyPos.y to MEM[1]
			         ADC14_MCTLN_EOS;          // Stop ADC from checking chanels once it reaches MEM[2]
	ADC14->IER0 = 0; // Don't want interrupts
	ADC14->CTL0 |= ADC14_CTL0_ENC;             // Enable Conversions
	ADC14->CTL0 |= ADC14_CTL0_SC;        // Sampling and conversion start
}

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
	NVIC_EnableIRQ(PORT5_IRQn);

	nrfClearStatus();
	nrfWReg(NRF_CONFIG_ADDR, NRF_CONFIG_RX_DR_IIE |
	                         NRF_CONFIG_EN_CRC |
	                         NRF_CONFIG_PWR_UP |
	                         NRF_CONFIG_PRIM_TX);

	nrfWReg(NRF_SETUP_RETR_ADDR, NRF_SETUP_RETR_ARD_1MS |
	                             NRF_SETUP_RETR_ARC_15);

	nrfWReg(NRF_RF_SETUP_ADDR, NRF_RF_SETUP_250KBPS |
	                           NRF_RF_SETUP_RF_PWR_0);
}

void setupLED(void) {
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
 * Helper functions
 */

inline void requestMode(opMode from, opMode to) {
	__disable_interrupts(); // Makes sure a forced change sticks in an ISR, as requested mode is either rejected here or takes effect immediately after this function.
	if (mode == from) mode = to;
	__enable_interrupts();
}

inline void forceMode(opMode newMode) {
	mode = newMode;
}

void printHelp(void) {
	printWrap("Please choose one of the following single character commands:", 0, 0, 0);
	printNewline();
	printWrap("d - Disable the transmitter.", 4, 8, 0);
	printWrap("e - Enable the transmitter.", 4, 8, 0);
	printWrap("i - Toggle printing of status reports", 4, 8, 0);
	printWrap("h - Re-print this help message.", 4, 8, 0);
	printWrap("p - Change preferences. Such as the intensity of the LED, the frequency of \npacket exchange, etc.", 4, 8, 0);
	printWrap("r - Range test, send NOP codes and report if they succeed or fail with the \ngreen and red LEDs, respectively.", 4, 8, 0);
	printWrap("s - Send command. You can send either a command or state. For a command, \nenter 0 to 63, for a state, you will be asked for a speed from 1 to 100, a direction to turn in, and the copies of the packet to send.", 4, 8, 0);
	printNewline();
}

void printInvalidCommand(uint8_t received) {
	char str[5];
	printString("That command is not registered. You entered: '");
	// Print printable characters
	if (printable(received)) printChar(received);
	printString("' (ASCII code = ");
	uInt8ToHexStr(received, str);
	printString(str);
	printChar(')');
	printNewline();
	printHelp();
}

inline void pauseStatusReports(void) {
	TIMER_A1->CTL &= ~TIMER_A_CTL_IE;
}

inline void resumeStatusReports(void) {
	TIMER_A1->CTL |= TIMER_A_CTL_IE;
}

void updateLED(opMode ledMode) {

	if (ledMode == MODE_SETUP ||
	    ledMode == MODE_CONNECTION_DROPPED ||
	    ledMode == MODE_DISABLED) {
		TIMER_A3->CTL |= TIMER_A_CTL_CLR;  // Reset
		TIMER_A3->CTL |= TIMER_A_CTL_IE;   // Need interrupt to change visuals
		pwm(0, CCR1);
		pwm(0, CCR2);
		pwm(0, CCR3);
	}
	else TIMER_A3->CTL &= ~TIMER_A_CTL_IE; // LED pwm doesn't change over time

	if (ledMode == MODE_NORMAL) {
		pwm(prefs.breakMode ? prefs.brightness : 0, CCR1); // Display the current setup
		pwm(prefs.fastMode  ? prefs.brightness : 0, CCR2);
		pwm(prefs.safeMode  ? prefs.brightness : 0, CCR3);
	}

	if (ledMode == MODE_RANGE_TEST) {
		if (flags.packetDropped) {
			pwm(prefs.brightness, CCR1);
			pwm(0, CCR2);
			pwm(0, CCR3);
		}
		else {
			pwm(0, CCR1);
			pwm(prefs.brightness, CCR2);
			pwm(0, CCR3);
		}
	}
}

void updateState(void) {
	if (P3->IN & BIT5) nextState.dir = DIR_LEFT;
	if (P5->IN & BIT1) nextState.dir = nextState.dir == DIR_LEFT ? DIR_STRAIT : DIR_RIGHT;
	nextState.speed = ADC14->MEM[1] >> 10;
}

void transmit(void) {
	if (nrfStatus.txDS || nrfStatus.txFull || nrfStatus.maxRT) { // Cannot send until TX data sent flag is cleared and the buffer is not empty
		nrfClearStatus();
		nrfUpdateStatus();
		nrfWaitForSPI();
	}
	switch (mode) {
	case MODE_SETUP:
	case MODE_NORMAL:
		if (flags.sendCommand) nrfTXCommand(nextCommand);
		else nrfTXState(nextState);
		packetsSent++;
		nextState.dir = DIR_STRAIT;
		nextState.speed = 0;

		if (flags.packetDropped) {
			droppedPackets++;
			requestMode(MODE_NORMAL, MODE_CONNECTION_DROPPED);
		}
		break;
	case MODE_DBUG:
		if (!packetsToSend) {
			flags.sendCommandDbug = 0;
			requestMode(MODE_DBUG, MODE_NORMAL);
		}
		if (flags.sendCommandDbug) nrfTXCommand(nextCommandDbug);
		else nrfTXState(nextStateDbug);
		packetsToSend--;

		if (flags.packetDropped) {
			droppedPackets++;
			requestMode(MODE_DBUG, MODE_CONNECTION_DROPPED);
		}
		break;
	case MODE_CONNECTION_DROPPED:
		if (!flags.packetDropped) requestMode(MODE_CONNECTION_DROPPED, MODE_NORMAL);
		// Fallthrough
	case MODE_RANGE_TEST:
		nrfTXCommand(CMD_NOP);
		break;
	}
}

/**
 * Interrupt handlers
 */

void TA1_N_IRQHandler(void) {
	TIMER_A1->CTL &= ~TIMER_A_CTL_IFG;
	flags.log = 1;
}

void TA2_N_IRQHandler(void) {
	TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;
	runAsync(&updateState);
	runAsync(&transmit);
}

void TA3_N_IRQHandler(void) {
	TIMER_A3->CTL &= ~TIMER_A_CTL_IFG;
	static uint8_t tics = 0;
	switch (mode) {
	case MODE_SETUP:
		// Make pwm a sine wave
		break;
	case MODE_CONNECTION_DROPPED:
		if (tics == 5) pwm(prefs.brightness, CCR1);
		else if (tics == 10) {
			tics = 0;
			pwm(0, CCR1);
		}
		tics++;
		break;
	case MODE_DISABLED:
		if (tics == 5) pwm(prefs.brightness, CCR3);
		else if (tics == 10) {
			tics = 0;
			pwm(0, CCR3);
		}
		tics++;
		break;
	}
}

void EUSCIA1_IRQHandler(void) {
	uint8_t byte;
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_RXIFG) {
		byte = EUSCI_A1_SPI->RXBUF;
		if (!flags.statusRead) {
			nrfStatus.reg = byte;
			if (nrfStatus.txDS) flags.packetDropped = 0;
			if (nrfStatus.maxRT) flags.packetDropped = 1;
			flags.statusRead = 1;
		}
		addToBuff(&miso, byte);
	}
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_TXIFG && EUSCI_A1_SPI->IE & EUSCI_A_IE_TXIE) {

		if (getFromBuff(&mosi, &byte) == ERR_NO) EUSCI_A1_SPI->TXBUF = byte;
		else EUSCI_A1_SPI->IE &= ~EUSCI_A_IE_TXIE;
	}
}

void PORT1_IRQHandler (void) {
	if (P1->IFG & BIT1) {
		pauseCommands();
		endRangeTestConfirm(mode == MODE_RANGE_TEST ? 'y' : 'n'); // Toggle range test mode
	}
	if (P1->IFG & BIT4) {
		// Left button pressed
	}
	P1->IFG = 0;
}

void PORT3_IRQHandler (void) {
	if (P3->IFG & BIT5) {
		nextState.dir = DIR_LEFT;
	}
	P3->IFG = 0;
}

void PORT5_IRQHandler (void) {
	if (P5->IFG & BIT1) {
		nextState.dir = DIR_RIGHT;
	}
	if (P5->IFG & BIT6) { // Got an interrupt, need to update status so rest of program know state
		runAsync(&nrfUpdateStatus);
	}
	P5->IFG = 0;
}









#else // Car









/**
 *  Type definitions
 */
typedef enum {
	MODE_NORMAL,             // Controlling car
	MODE_SAFE,
	MODE_CONNECTION_DROPPED, // Lost connection unexpectedly
	MODE_DISABLED            // Changing settings
} opMode;

/**
 * Global variables
 */
static struct {
	FLAG(connectionDropped);
	FLAG(statusRead);
	FLAG(expectingRX);
	FLAG(log);
} flags = {0};
uint8_t lastRX = 0;
command lastCommand = CMD_NOP;
state lastState = {0};
opMode mode = MODE_NORMAL;
uint32_t packetsReceived = 0;

/**
 * Helper functions
 */
inline void requestMode(opMode from, opMode to);
inline void forceMode(opMode to);
void resetWatchdog(void);

/**
 * Setup functions
 */
void setupLogging(void);
void setupTimers(void);
void setupInputs(void);
void setupNRF(void);

void main(void) { // Responsible for setup and user IO.
	char str[21]; // Enough for 2^64 + '\0' as well as 0xFFFFFF... + '\0'
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog timer
	setCPUFreq(FREQ_1_5);
	initAsync();
	setupLogging();
	resetWatchdog();
	NVIC_EnableIRQ(WDT_A_IRQn);
	setupTimers();
	setupInputs();
	setupNRF();
	log("Setup complete");

	__enable_interrupts();
	while(1) {
		switch (mode) {
		case MODE_NORMAL:
//			resumeStatusReports();
			while (mode == MODE_NORMAL) {
//				pauseCommands();

				if (flags.log) {
					log("Status Report:");
					pauseCommands();
					printString("Packets received: ");
					uIntToStr(packetsReceived, str, 0);
					printString(str);
					printNewline();
					printString("Last packet:      ");
					uInt8ToHexStr(lastRX, str);
					printString(str);
					printNewline();
					printString("NRF status:       ");
					uInt8ToHexStr(nrfStatus.reg, str);
					printString(str);
					printNewline();
					packetsReceived = 0;
					flags.log = 0;
				}

				if (flags.connectionDropped) requestMode(MODE_NORMAL, MODE_CONNECTION_DROPPED);

//				readyForCommands();
			}
			break;
		case MODE_CONNECTION_DROPPED:
			log("Connection dropped");
			log("Waiting for reconnect");
			while (mode == MODE_CONNECTION_DROPPED) {
				if (!flags.connectionDropped) requestMode(MODE_CONNECTION_DROPPED, MODE_NORMAL);
			}
			log("Connection restored");
			break;
		}
	}
}

/**
 * Helper functions
 */

inline void requestMode(opMode from, opMode to) {
	__disable_interrupts(); // Makes sure a forced change sticks in an ISR, as requested mode is either rejected here or takes effect immediately after this function.
	if (mode == from) mode = to;
	__enable_interrupts();
}

inline void forceMode(opMode newMode) {
	mode = newMode;
}

void resetWatchdog(void) {
	WDT_A->CTL = WDT_A_CTL_PW |         // Password, needed to modify without resetting device
                 WDT_A_CTL_SSEL__ACLK | // Use 32 khz clock
	             WDT_A_CTL_TMSEL |      // Interval mode
	             WDT_A_CTL_CNTCL |      // Clear the counter
				 WDT_A_CTL_IS_4;        // 250 ms interval
}

/**
 * Setup functions
 */

void setupLogging(void) {
	configLogging(EUSCI_A0, 115200);
	P1->SEL0 |= BIT2 | BIT3;                    // 2 = RX 3 = TX
	P1->SEL1 &= ~(BIT2 | BIT3);                 // Set to primary function (01)
	startLogging(EUSCIA0_IRQn);
//	enableCommands(&commandHandler);
}

void setupTimers(void) {
	// 1 Hz logging interrupt
	initTimerA(TIMER_A1, 1);
	TIMER_A1->CTL |= TIMER_A_CTL_IE;
	NVIC_EnableIRQ(TA1_N_IRQn);
}

void setupInputs(void) {
	// Sensor
	// ...
}

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
	NVIC_EnableIRQ(PORT5_IRQn);

	nrfClearStatus();
	nrfWReg(NRF_CONFIG_ADDR, NRF_CONFIG_TX_DS_IIE |
	                         NRF_CONFIG_EN_CRC |
	                         NRF_CONFIG_PWR_UP |
	                         NRF_CONFIG_PRIM_RX);

	nrfWReg(NRF_SETUP_RETR_ADDR, NRF_SETUP_RETR_ARD_1MS |
	                             NRF_SETUP_RETR_ARC_15);

	nrfWReg(NRF_RF_SETUP_ADDR, NRF_RF_SETUP_250KBPS |
	                           NRF_RF_SETUP_RF_PWR_0);

	nrfWReg(NRF_RX_PW_P0_ADDR, 1);
	nrfEnCE();
}

/**
 * Interrupt handlers
 */

void TA1_N_IRQHandler(void) {
	TIMER_A1->CTL &= ~TIMER_A_CTL_IFG;
	flags.log = 1;
}

void EUSCIA1_IRQHandler(void) {
	uint8_t byte;
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_RXIFG) {
		byte = EUSCI_A1_SPI->RXBUF;

		if (!flags.statusRead) {
			nrfStatus.reg = byte;
			flags.statusRead = 1;
			if (nrfStatus.rxDR) {
				runAsync(&nrfReadRX);
				flags.connectionDropped = 0;
				resetWatchdog();
			}
		}
		else if (flags.expectingRX) {
			packetsReceived++;
			flags.expectingRX = 0;
			lastRX = byte;
			decodePacket(byte, &lastCommand, &lastState);
		}
		addToBuff(&miso, byte);
	}
	if (EUSCI_A1_SPI->IFG & EUSCI_A_IFG_TXIFG && EUSCI_A1_SPI->IE & EUSCI_A_IE_TXIE) {
		if (getFromBuff(&mosi, &byte) == ERR_NO) EUSCI_A1_SPI->TXBUF = byte;
		else EUSCI_A1_SPI->IE &= ~EUSCI_A_IE_TXIE;
	}
}

void WDT_A_IRQHandler(void) {
	flags.connectionDropped = 1;
}

void PORT5_IRQHandler (void) {
	if (P5->IFG & BIT6) { // Got an interrupt, need to update status so rest of program know state
		runAsync(&nrfUpdateStatus);
	}
	P5->IFG = 0;
}

#endif

/**
 * Global function implementations
 */

/**
 * Helper functions
 */

inline float sin(uint8_t theta)  {
	float sine = 0;
	uint8_t sign = theta % 100 > 50 ? -1 : 1;
	theta %= 50;
	if (theta > 25) sine = sines[50 - theta];
	else sine = sines[theta];
	return sign * sine;
}

uint8_t encodeState(state stateToEncode) {
	return ((int8_t) stateToEncode.speed) << SPEED_OFS | stateToEncode.dir << DIR_OFS;
}

uint8_t encodeCommand(command commandToEncode) {
	return commandToEncode << CMD_OFS | CMD_ENCODING;
}

void decodePacket(uint8_t packet, command * commandContainer, state * stateContainer) {
	if (packet & DIR_MASK == CMD_ENCODING) *commandContainer = (command) ((packet & CMD_MASK) >> CMD_OFS);
	else {
		stateContainer->dir = (direction) ((packet & DIR_MASK) >> DIR_OFS);
		stateContainer->speed = (packet & SPEED_MASK) >> SPEED_OFS;
	}
}

/**
 * NRF functions
 */
inline void nrfEnCE(void) {
	P5->OUT |= BIT7;
}

inline void nrfDsCE(void) {
	P5->OUT &= ~BIT7;
}
inline void nrfWaitForSPI(void) {
	while (EUSCI_A1_SPI->STATW & EUSCI_A_STATW_BUSY || !buffEmpty(&mosi));
	flags.statusRead = 0;
}

inline void nrfSendSPI(void) {
	EUSCI_A1_SPI->IE |= EUSCI_A_IE_TXIE;
}

inline void nrfUpdateStatus(void) {
	nrfWaitForSPI();
	__disable_interrupts();
	addToBuff(&mosi, NRF_NOP);
	nrfSendSPI();
	__enable_interrupts();
}

inline void nrfFlushTXFIFO(void) {
	nrfWaitForSPI();
	__disable_interrupts();
	addToBuff(&mosi, NRF_FLUSH_TX);
	nrfSendSPI();
	__enable_interrupts();
}

inline void nrfFlushRXFIFO(void) {
	nrfWaitForSPI();
	__disable_interrupts();
	addToBuff(&mosi, NRF_FLUSH_RX);
	nrfSendSPI();
	__enable_interrupts();
}

inline void nrfRReg(uint8_t reg) {
	nrfWaitForSPI();
	__disable_interrupts();
	addToBuff(&mosi, NRF_R_REGISTER(reg));
	addToBuff(&mosi, NRF_WAIT_FOR_DATA);
	nrfSendSPI();
	__enable_interrupts();
}

inline void nrfWReg(uint8_t reg, uint8_t payload) {
	nrfWaitForSPI();
	__disable_interrupts();
	addToBuff(&mosi, NRF_W_REGISTER(reg));
	addToBuff(&mosi, payload);
	nrfDsCE(); // Needed when writing registers
	nrfSendSPI();
	__enable_interrupts();
	nrfWaitForSPI();
	nrfEnCE();
}

inline void nrfReadRX(void) {
	nrfWaitForSPI();
	__disable_interrupts();
	flags.expectingRX = 1;
	addToBuff(&mosi, NRF_R_RX_PAYLOAD);
	addToBuff(&mosi, NRF_WAIT_FOR_DATA);
	nrfSendSPI();
	__enable_interrupts();
	nrfClearStatus();
}

inline void nrfTXState(state stateToTX) {
	nrfWaitForSPI();
	__disable_interrupts();
	addToBuff(&mosi, NRF_W_TX_PAYLOAD);
	addToBuff(&mosi, encodeState(stateToTX));
	nrfEnCE(); // Needed when transmitting
	nrfSendSPI();
	__enable_interrupts();
}

inline void nrfTXCommand(command commandToTX) {
	nrfWaitForSPI();
	__disable_interrupts();
	addToBuff(&mosi, NRF_W_TX_PAYLOAD);
	addToBuff(&mosi, encodeCommand(commandToTX));
	nrfEnCE(); // Needed when transmitting
	nrfSendSPI();
	__enable_interrupts();
}

inline void nrfClearStatus(void) {
	nrfFlushRXFIFO();
	nrfFlushTXFIFO();
	nrfWReg(NRF_STATUS_ADDR, NRF_STATUS_RX_DR |
	                         NRF_STATUS_TX_DS |
	                         NRF_STATUS_MAX_RT |
	                         NRF_STATUS_RX_P_NO);
}


















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
	// Clear interrupt flagsOLD
	addToBuff(&mosiBuffRX, NRF_W_REGISTER(NRF_STATUS_ADDR));
	addToBuff(&mosiBuffRX, NRF_STATUS_RX_DR |
			               NRF_STATUS_TX_DS |
						   NRF_STATUS_MAX_RT |
						   NRF_STATUS_RX_P_NO);
	NRF_SEND();
}

void clearNRFTXFlags(void) {
	// Clear interrupt flagsOLD
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
	logIS("Clearing flagsOLD");
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
