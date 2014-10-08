#ifndef __UART_H 
#define __UART_H

#include "chip.h"

//Transmit buffer that will be used on the rest of the system
extern RINGBUFF_T uart0TX;
extern RINGBUFF_T uart0RX;

extern RINGBUFF_T uart3RX;

extern volatile bool clearToSend; //status of the CTS flag

#define CTS_BUFFER_THRESHOLD			128


/**
 * It initializes the selected UART peripheral.
 * @param UARTx Pointer to the UART peripheral
 * @param Baudrate Baud rate to be used
 */
extern void UARTsInit(void);

/**
 * Prints through the UART the version string of the firmware.
 */
extern void UARTShowVersion(void);

/**
 * Parses the character received
 * @param newChar new character received
 */
extern void UART0ParseNewChar();



#endif /* end __UART_H */
/*****************************************************************************
 **                            End Of File
 ******************************************************************************/
