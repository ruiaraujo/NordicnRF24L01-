#include <string.h>
#include "chip.h"
#include "utils.h"
#include "uart.h"
#include "xprintf.h"
#include "error.h"
#include "pinout.h"
#include "nRF24L01P.h"
#include <stdbool.h>
#include <ctype.h>

#define SOFTWARE_VERSION "0.1"

#define DEFAULT_BAUDRATE_UART0			(4000000)

#define TX_BUFFER_SIZE_BITS		(12)
#define TX_BUFFER_SIZE			(1<<TX_BUFFER_SIZE_BITS)

#define RX_BUFFER_SIZE_BITS		(12)
#define RX_BUFFER_SIZE			(1<<RX_BUFFER_SIZE_BITS)

//Transmit buffer that will be used on the rest of the system
RINGBUFF_T uart0TX;
RINGBUFF_T uart0RX;

char uart0TxBuffer[TX_BUFFER_SIZE];
char uart0RxBuffer[RX_BUFFER_SIZE];

#define UART_COMMAND_LINE_MAX_LENGTH  	128

// *****************************************************************************

unsigned char commandLine[UART_COMMAND_LINE_MAX_LENGTH];
uint32_t commandLinePointer;
uint32_t enableUARTecho;	 // 0-no cmd echo, 1-only cmd reply, 2-all visible

// *****************************************************************************
#define UARTReturn()	   xputc('\n')

/* The rate at which data is sent to the queue, specified in milliseconds. */

void UART0WriteChar(char pcBuffer) {
	while (Chip_UART_SendRB(LPC_UART0, &uart0TX, &pcBuffer, 1) == 0) {
		; //Wait for the M0 core to move some char out
	}
}

void UART0_IRQHandler(void) {
	Chip_UART_IRQRBHandler(LPC_UART0, &uart0RX, &uart0TX);
}

void UARTsInit() {
	memset(commandLine, 0, UART_COMMAND_LINE_MAX_LENGTH);
	commandLinePointer = 0;
	enableUARTecho = 1;
	xdev_out(UART0WriteChar);
	Chip_UART_Init(LPC_UART0);
	Chip_UART_ConfigData(LPC_UART0, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_UART0, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_IntEnable(LPC_UART0, UART_IER_RBRINT);
	Chip_UART_SetBaudFDR(LPC_UART0, DEFAULT_BAUDRATE_UART0);
	Chip_UART_TXEnable(LPC_UART0);
	NVIC_EnableIRQ(UART0_IRQn);
	RingBuffer_Init(&uart0TX, uart0TxBuffer, 1, TX_BUFFER_SIZE);
	RingBuffer_Init(&uart0RX, uart0RxBuffer, 1, RX_BUFFER_SIZE);
}

// *****************************************************************************

// *****************************************************************************
void UARTShowVersion(void) {
	xputs("\nNordicLPC1759, V" SOFTWARE_VERSION " " __DATE__ ", " __TIME__ "\n");
}

// *****************************************************************************
static void UARTShowUsage(void) {

	UARTShowVersion();

	UARTReturn();
	xputs("Supported Commands:\n");
	UARTReturn();

	xputs(" !I                - init Nordic\n");
	xputs(" !It               - set Transmit mode\n");
	xputs(" !Tx               - transmit x\n");
	UARTReturn();

	xputs(" !U=x                - set baud rate to x\n");
	xputs(" !U[0,1]             - UART echo mode (none, all)\n");
	UARTReturn();

	xputs(" ??                  - display (this) help\n");
	UARTReturn();
}

// *****************************************************************************
static uint32_t parseUInt32(unsigned char **c) {
	uint32_t ul = 0;
	while (((**c) >= '0') && ((**c) <= '9')) {
		ul = 10 * ul;
		ul += ((**c) - '0');
		(*(c))++;
	}
	return (ul);
}

// *****************************************************************************
// * ** parseGetCommand ** */
// *****************************************************************************
static void UARTParseGetCommand(void) {

	switch (commandLine[1]) {

	case '?':
		UARTShowUsage();
		break;
	default:
		xputs("Get: parsing error\n");
	}
}

// *****************************************************************************
// * ** parseSetCommand ** */
// *****************************************************************************
static void UARTParseSetCommand(void) {
	switch (commandLine[1]) {
	case 'I':
	case 'i': {
		nRF24L01PInit();
		unsigned char *c = commandLine + 2;
		if (*c == 't' || *c == 'T') {
			RFdisable();
			RFsetTransmitMode();
			RFenable();
		}
		break;
	}
	case 'T':
	case 't': {
		RFpushDataToBuffer(commandLine + 2, commandLinePointer - 2);
		break;
	}
	case 'U':
	case 'u': {
		unsigned char *c;
		long baudRate;
		c = commandLine + 2;
		if (*c >= '0' && *c <= '2') {
			enableUARTecho = (*c - '0');
			break;
		}
		c++;
		baudRate = parseUInt32(&c);
		while ((LPC_UART0->LSR & UART_LSR_TEMT) == 0) {
		};		   // wait for UART to finish data transfer
		xprintf("-U=%d\n", baudRate);
		timerDelayMs(100);
		if (Chip_UART_SetBaudFDR(LPC_UART0, baudRate) == 0) {
			xprintf("Failed to switch Baud Rate to %d Baud!\n", baudRate);
		}
		break;
	}
	default:
		xputs("Set: parsing error\n");
	}
}

// *****************************************************************************
// * ** parseRS232CommandLine ** */
// *****************************************************************************
static void parseRS232CommandLine(void) {

	switch (commandLine[0]) {
	case '?':
		UARTParseGetCommand();
		break;
	case '!':
		UARTParseSetCommand();
		break;
	default:
		xputs("?\n");
	}
}

// *****************************************************************************
// * ** RS232ParseNewChar ** */
// *****************************************************************************
void UART0ParseNewChar() {
	unsigned char newChar;
	RingBuffer_Pop(&uart0RX, &newChar);
	switch (newChar) {
	case 8:			// backspace
		if (commandLinePointer > 0) {
			commandLinePointer--;
			if (enableUARTecho) {
				xprintf("%c %c", 8, 8);
			}
		}
		break;

	case 10:
	case 13:
		if (enableUARTecho) {
			UARTReturn();
		}
		if (commandLinePointer > 0) {
			commandLine[commandLinePointer] = 0;
			parseRS232CommandLine();
			commandLinePointer = 0;
		}
		break;

	default:
		if (newChar & 0x80) {
			return; //only accept ASCII
		}
		if (commandLinePointer < UART_COMMAND_LINE_MAX_LENGTH - 1) {
			if (enableUARTecho) {
				xputc(newChar);	  		   	// echo to indicate char arrived
			}
			commandLine[commandLinePointer++] = newChar;
		} else {
			commandLinePointer = 0;
			commandLine[commandLinePointer++] = newChar;
		}
	}  // end of switch

}  // end of rs232ParseNewChar

