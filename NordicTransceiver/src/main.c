#include <cr_section_macros.h>
#include "chip.h"
#include "uart.h"
#include "pinout.h"
#include "nRF24L01P.h"
#include "xprintf.h"

/**
 * The Systick handler is used for a lot more tasks than sensor timing.
 * It also provides a timer for decaying for the motor velocity, motor control
 * and second timer used for the LED blinking and Retina event rate.
 */
volatile uint8_t toggleLed0 = 0;
void SysTick_Handler(void) {
	static uint16_t second_timer = 0;
	if (nRF24L01P.transmissionTimeout > 0) {
		nRF24L01P.transmissionTimeout--;
	}
	if (++second_timer >= 1000) {
		second_timer = 0;
		toggleLed0 = 1;
	}
}

char RFreception[NRF24L01P_RX_FIFO_SIZE + 1];

int main(void) {
	UARTsInit();
	UARTShowVersion();
	nRF24L01PInit();
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, LED_PORT, LED_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, LED_PORT, LED_PIN);

	uint32_t load = Chip_Clock_GetSystemClockRate() / 1000 - 1;
	if (load > 0xFFFFFF) {
		load = 0xFFFFFF;
	}
	SysTick->LOAD = load;
	SysTick->CTRL |= 0x7;	//enable the Systick
	while (1) {
		if (RingBuffer_GetCount(&uart0RX)) {
			UART0ParseNewChar();
		}
		if (toggleLed0) {
			toggleLed0 = 0;
			Chip_GPIO_SetPinToggle(LPC_GPIO, LED_PORT, LED_PIN);
		}
		if (nRF24L01P.transmissionTimeout == 0) {
			RFmoveBufferToTransmission(NRF24L01P_PIPE_P0);
			nRF24L01P.transmissionTimeout = -1;
		}
		//New nordic interrupt active low
		if (Chip_GPIO_ReadPortBit(LPC_GPIO, NORDIC_IRQ_PORT, NORDIC_IRQ_PIN) == 0) {
			int status = RFgetStatusRegister();
			if (status & NRF24L01P_STATUS_TX_DS) {
				//xputs("Finished tx\n");
				RFsetRegister(NRF24L01P_REG_STATUS, NRF24L01P_STATUS_TX_DS);
				if (nRF24L01P.mode & NRF24L01P_MODE_RX) {
					nRF24L01P.ACKPayloadsinTXFIFO--; // If we are receiving, this interrupt indicates a
				}
				if (RFisTXBufferEmpty()) {
					RFsetNormalMode();
				} else {
					RFmoveBufferToTransmission(NRF24L01P_PIPE_P0);
				}
			}
			if (status & NRF24L01P_STATUS_MAX_RT) {
				//xputs("Retx limit\n");
				RFsetRegister(NRF24L01P_REG_STATUS, NRF24L01P_STATUS_MAX_RT);
				if (RFisTXBufferEmpty()) {
					RFsetNormalMode();
				} else {
					RFmoveBufferToTransmission(NRF24L01P_PIPE_P0);
				}
			}
			if (status & NRF24L01P_STATUS_RX_DR) {
				//xputs("new packet\n");
				do {
					int rxPayloadsize = RFgetRXPayloadSize();
					if (rxPayloadsize < 0) {
						RFFlushRXFIFO();
					} else {
						RFRXFIFOread(RFreception, rxPayloadsize);
						RFreception[rxPayloadsize] = '\0';
						xputs(RFreception);
						xputc('\n');
					}
					RFsetRegister(NRF24L01P_REG_STATUS, NRF24L01P_STATUS_RX_DR);
				} while (!RFisRXFIFOEmpty());
			}
		}
	}
	return 0;
}
