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
	CMD_MODE_SLOW_EN,  // Enable halved speeds
	CMD_MODE_SLOW_DS,  // Disable halved speeds
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
		"Disable safe mode"
};
char * directionDescriptions[] = {
		"Strait",
		"Right",
		"Left"
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
inline state decodeState(uint8_t packet);
inline command decodeCommand(uint8_t packet);
inline uint8_t isCommand(uint8_t packet);

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
inline void nrfTXState(state stateToTX);
inline void nrfTXCommand(command commandToTX);
inline void nrfClearStatus(void);










#ifdef TRANSMITTER

/**
 * Main code for transmitter
 */

#define NRF_CE_PORT      P10
#define NRF_CE_PIN       BIT4
#define JOY_X_CENT       2
#define JOY_Y_CENT       -2
#define JOY_JITTER       1
#define JOY_LEFT_THRESH  -10
#define JOY_RIGHT_THRESH 10

/**
 *  Type definitions
 */
typedef enum {
	MODE_SETUP,              // Changing settings
	MODE_NORMAL,             // Controlling car
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
	FLAG(sendState);
	FLAG(sendCommand);
	FLAG(expectingRX);
	FLAG(log);
} flags = {0};
static uint32_t packetsSent = 0;
static uint32_t droppedPackets = 0;
static state nextState;
static state cliState;
static command nextCommand;
static uint32_t packetsToSend = 0;
static struct {
	float brightness;
	FLAG(breakMode);
	FLAG(slowMode);
	FLAG(safeMode);
	FLAG(joyTurns);
	FLAG(dbg);
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
					if (packetsToSend) {
						printString("Packets left:    ");
						uIntToStr(packetsToSend, str, 0);
						printString(str);
						printNewline();
					}
					if (prefs.dbg) {
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
					}
					printNewline();
					packetsSent = 0;
					droppedPackets = 0;
					flags.log = 0;
					readyForCommands();
				}

			}
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
			log("Looking for receiver...");
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
	nextCommand = (command) option;
	flags.sendCommand = 1;
	transmit();
	readyForCommands();
}

void packetsToSendHandler(int64_t number) {
	packetsToSend = number;
	readyForCommands();
}

void speedHandler(int64_t number) {
	cliState.speed = number;
	printWrap("Choose a number of packets to send", 0, 0, 0);
	getNumberInRange(&packetsToSendHandler, 0, INT64_MAX);
}

void directionOptionHandler(uint32_t option) {
	cliState.dir = (direction) option;
	printWrap("Choose a speed from -32 to 31.", 0, 0, 0);
	getNumberInRange(&speedHandler, -32, 31);
}

void commandOrState(char received) {
	switch (received) {
	case 'c':
		printWrap("Choose a command", 0, 0, 0);
		getOption(&commandOptionHandler, sizeof commandDescriptions / sizeof (char *), commandDescriptions);
		break;
	case 's':
		printWrap("Choose a direction", 0, 0, 0);
		getOption(&directionOptionHandler, sizeof directionDescriptions / sizeof (char *), directionDescriptions);
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
	case 'g':
		pauseCommands();
		if (prefs.dbg) {
			prefs.dbg = 0;
			printWrap("Debugging mode disabled", 0, 0, 0);
		}
		else {
			prefs.dbg = 1;
			printWrap("Debugging mode enabled", 0, 0, 0);
		}
		readyForCommands();
		break;
	case 'i':
		pauseCommands();
		if (TIMER_A1->CTL & TIMER_A_CTL_IE) {
			pauseStatusReports();
			printWrap("Status reports disabled", 0, 0, 0);
		}
		else {
			resumeStatusReports();
			printWrap("Status reports enabled", 0, 0, 0);
		}
		readyForCommands();
		break;
	case 'h':
		pauseCommands();
		printHelp();
		readyForCommands();
		break;
	case 'p':
		// TODO: Preferences
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

	// Turn off LCD
	P2->OUT &= ~BIT6;
	P2->DIR |= BIT6;

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
	mapPin(PORT_7, PIN_5, PMAP_UCA1STE);
	mapPin(PORT_7, PIN_4, PMAP_UCA1CLK);
	mapPin(PORT_7, PIN_7, PMAP_UCA1SIMO);
	mapPin(PORT_7, PIN_6, PMAP_UCA1SOMI);
	startSPI(EUSCI_A1_SPI);
	EUSCI_A1_SPI->IE |= EUSCI_A_IE_TXIE | EUSCI_A_IE_RXIE;
	NVIC_EnableIRQ(EUSCIA1_IRQn);

	// CE
	P10->OUT &= ~BIT4;
	P10->DIR |= BIT4;
	// IRQ
	P6->DIR &= ~BIT3;
	P6->IFG = 0;
	P6->IES |= BIT3;            // Interrupt fires on high to low transition
	P6->IE |= BIT3;
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
	printWrap("g - Toggle debug mode.", 4, 8, 0);
	printWrap("i - Toggle status reports", 4, 8, 0);
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
		pwm(prefs.safeMode  ? prefs.brightness : 0, CCR2);
		pwm(prefs.slowMode  ? prefs.brightness : 0, CCR3);
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
	int8_t x = ADC14->MEM[0] >> 8; // 8 for register offset...
	int8_t y = ADC14->MEM[1] >> 8;
	x >>= 2; //  ... 2 for 6 bit width. Done separately for sign extension.
	y >>= 2;
	if (prefs.joyTurns) {
		if (x <= JOY_LEFT_THRESH) nextState.dir = DIR_LEFT;
		else if (x >= JOY_RIGHT_THRESH) nextState.dir = DIR_RIGHT;
		else nextState.dir = DIR_STRAIT;
	}
	else { // Using buttons
		if (P5->IN & BIT1 && P3->IN & BIT5 || !(P5->IN & BIT1) && !(P3->IN & BIT5)) nextState.dir = DIR_STRAIT;
		else if (P3->IN & BIT5) nextState.dir = DIR_RIGHT;
		else nextState.dir = DIR_LEFT;
	}
	nextState.speed = y < JOY_Y_CENT - JOY_JITTER  || y > JOY_Y_CENT + JOY_JITTER? y : 0;
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
		if (flags.sendCommand) {
			nrfTXCommand(nextCommand);
			flags.sendCommand = 0;
		}
		else {
			updateState();
			// Input from controller trumps state from command line
			if (nextState.dir != DIR_STRAIT || nextState.speed != 0) packetsToSend = 0;
			if (packetsToSend) {
				nrfTXState(cliState);
				packetsToSend--;
			}
			else nrfTXState(nextState);
		}
		packetsSent++;
		nextState.dir = DIR_STRAIT;
		nextState.speed = 0;

		if (flags.packetDropped) {
			droppedPackets++;
			requestMode(MODE_NORMAL, MODE_CONNECTION_DROPPED);
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
		forceMode(MODE_DISABLED);
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








#define NRF_CE_PORT P5
#define NRF_CE_PIN  BIT7
#define FL          BIT4
#define FR          BIT5
#define BL          BIT6
#define BR          BIT7
#define TURN        BIT0
#define LEFT        BIT1

/**
 *  Type definitions
 */
typedef enum {
	MODE_NORMAL,             // Controlling car
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
	FLAG(dbg);
} flags = {0};
static struct {
	FLAG(breakMode);
	FLAG(slowMode);
	FLAG(safeMode);
} prefs = {0, 0, 0};
static uint8_t lastRX = 0;
static command lastCommand = CMD_NOP;
static state lastState = {0};
static opMode mode = MODE_NORMAL;
static uint32_t packetsReceived = 0;

/**
 * Helper functions
 */
inline void requestMode(opMode from, opMode to);
inline void forceMode(opMode to);
void resetWatchdog(void);
inline void nrfReadRX(void);
inline void setState(state newState);

/**
 * Setup functions
 */
void setupLogging(void);
void setupTimers(void);
void setupIO(void);
void setupNRF(void);

void main(void) { // Responsible for setup and user IO.
	char str[21]; // Enough for 2^64 + '\0' as well as 0xFFFFFF... + '\0'
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog timer
	setCPUFreq(FREQ_1_5);
	initAsync();
	setupLogging();
	resetWatchdog(); // Starts it up
	NVIC_EnableIRQ(WDT_A_IRQn);
	setupTimers();
	setupIO();
	setupNRF();
	log("Setup complete");

	__enable_interrupts();
	while(1) {
		switch (mode) {
		case MODE_NORMAL:
			while (mode == MODE_NORMAL) {
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
					printString("Last speed:       ");
					intToStr(lastState.speed, str, 0);
					printString(str);
					printNewline();
					printString("Last direction:   ");
					printString(directionDescriptions[lastState.dir]);
					printNewline();
					printString("Last command:     ");
					printString(commandDescriptions[lastCommand]);
					printNewline();
					if (flags.dbg) {
						// TODO: Also print state of sensors and current settings
						printString("NRF status:       ");
						uInt8ToHexStr(nrfStatus.reg, str);
						printString(str);
						printNewline();
					}
					printNewline();
					packetsReceived = 0;
					flags.log = 0;
				}
			}
			break;
		case MODE_CONNECTION_DROPPED:
			log("Connection dropped");
			log("Waiting for reconnect");
			while (mode == MODE_CONNECTION_DROPPED) {
				setState((state){0, DIR_STRAIT});
				if (!flags.connectionDropped) requestMode(MODE_CONNECTION_DROPPED, MODE_NORMAL);
			}
			log("Connection restored");
			break;
		case MODE_DISABLED:
			log("Movement disabled");
			while (mode == MODE_DISABLED) {
				setState((state){0, DIR_STRAIT});
			}
			log("Movement Enabled");
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

inline void setState(state newState) {
	if (mode == MODE_DISABLED || mode == MODE_CONNECTION_DROPPED) newState = (state){0, DIR_STRAIT};
	float speedDutyCycle = 0;
	switch (newState.dir) {
	case DIR_STRAIT:
		P5->OUT &= ~TURN;
		break;
	case DIR_RIGHT:
		P5->OUT |= TURN;
		P5->OUT &= ~LEFT;
		break;
	case DIR_LEFT:
		P5->OUT |= TURN | LEFT;
		break;
	}
	if (newState.speed == 0) {
		pwm(prefs.breakMode, CCR1); // Braking / Coasting
		P3->OUT |= BIT6;
		P5->OUT |= BIT5;
	}
	else if (newState.speed > 0) {
		speedDutyCycle = (float) newState.speed / 31;
		pwm(speedDutyCycle * speedDutyCycle, CCR1); // Braking / Coasting
		P3->OUT |= BIT6;
		P5->OUT &= ~BIT5;
	}
	else {
		speedDutyCycle = (float) newState.speed / -32;
		pwm(speedDutyCycle * speedDutyCycle, CCR1); // Braking / Coasting
		P3->OUT &= ~BIT6;
		P5->OUT |= BIT5;
	}
}

inline void resetWatchdog(void) {
	WDT_A->CTL = WDT_A_CTL_PW |         // Password, needed to modify without resetting device
                 WDT_A_CTL_SSEL__ACLK | // Use 32 khz clock
	             WDT_A_CTL_TMSEL |      // Interval mode
	             WDT_A_CTL_CNTCL |      // Clear the counter
				 WDT_A_CTL_IS_5;        // 250 ms interval
}

void receive(void) {
	if (isCommand(lastRX)) {
		lastCommand = decodeCommand(lastRX);
		switch (lastCommand) {
		case CMD_NOP:           // No op. Could be used to test for connection.
			setState((state){0, DIR_STRAIT});
			break;
		case CMD_LIGHTS_EN:     // Turn on all lights
			// TODO
			break;
		case CMD_LIGHTS_DI:     // Turn off all lights
			// TODO
			break;
		case CMD_LIGHTS_BLINK:  // Blink lights
			// TODO
			break;

		case CMD_MOVE_EN:       // Enable movement
			forceMode(MODE_NORMAL);
			break;
		case CMD_MOVE_DS:       // Disable movement
			forceMode(MODE_DISABLED);
			break;

		case CMD_MODE_BREAK_EN: // Enable breaking on stop
			prefs.breakMode = 1;
			break;
		case CMD_MODE_BREAK_DS: // Disable breaking on stop
			prefs.breakMode = 0;
			break;
		case CMD_MODE_SLOW_EN:  // Enable speed governing
			prefs.slowMode = 1;
			break;
		case CMD_MODE_SLOW_DS:  // Disable speed governing
			prefs.slowMode = 0;
			break;
		case CMD_MODE_SAFE_EN:  // Enable sensors
			// TODO
			break;
		case CMD_MODE_SAFE_DS:  // Disable sensors
			// TODO
			break;
		}
	}
	else {
		lastState = decodeState(lastRX);
		if (prefs.slowMode) lastState.speed /= 2;
		setState(lastState);
	}
	packetsReceived++;
}

inline void nrfReadRX(void) {
	nrfUpdateStatus();
	nrfWaitForSPI();
	if (nrfStatus.rxDR) {
		// Received something, connection verified.
		flags.connectionDropped = 0;
		resetWatchdog();
		// Get the RX
		__disable_interrupts();
		flags.expectingRX = 1;
		addToBuff(&mosi, NRF_R_RX_PAYLOAD);
		addToBuff(&mosi, NRF_WAIT_FOR_DATA);
		nrfSendSPI();
		__enable_interrupts();
		nrfWaitForSPI();
	}
	nrfClearStatus();
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

void setupIO(void) {
	state s = {
		.dir = DIR_STRAIT,
		.speed = 0
	};
	// Lights
	P6->OUT |= FL | FR | BL | BR;
	P6->DIR |= FL | FR | BL | BR;

	// Motor driver
	// PWM timer
	initTimerA(TIMER_A0, 500);
	// PWM Pins
	P3->DIR |= BIT6 | BIT7;
	mapPin(PORT_3, PIN_7, PMAP_TA0CCR1A);
	P5->DIR |= BIT5;

	// Steering
	P5->DIR |= TURN | LEFT;

	setState(s); // Zero state
	// Sensor
	// TODO ...
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
		}
		else if (flags.expectingRX) {
			flags.expectingRX = 0;
			lastRX = byte;
			runAsync(&receive);
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
	requestMode(MODE_NORMAL, MODE_CONNECTION_DROPPED);
	setState((state){0, DIR_STRAIT});
}

void PORT5_IRQHandler (void) {
	if (P5->IFG & BIT6) { // Got an interrupt, need to check the RX
		runAsync(&nrfReadRX);
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

inline state decodeState(uint8_t packet) {
	state s = {
		.speed = (packet & SPEED_MASK) >> SPEED_OFS,
		.dir = (direction) ((packet & DIR_MASK) >> DIR_OFS)
	};
	return s;
}
inline command decodeCommand(uint8_t packet) {
	return (command) ((packet & CMD_MASK) >> CMD_OFS);
}

inline uint8_t isCommand(uint8_t packet) {
	return (packet & DIR_MASK) == CMD_ENCODING;
}

/**
 * NRF functions
 */
inline void nrfEnCE(void) {
	NRF_CE_PORT->OUT |= NRF_CE_PIN;
}

inline void nrfDsCE(void) {
	NRF_CE_PORT->OUT &= ~NRF_CE_PIN;
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
