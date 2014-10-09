#include <cr_section_macros.h>
#include "chip.h"
#include "uart.h"
#include "pinout.h"
#include "nRF24L01P.h"
#include "xprintf.h"
#include <stdbool.h>

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

int throughMode = 0;
static unsigned char buffer[64];
static unsigned char fakeEvent[2];

int main(void) {

	unsigned short eventCounter = 0;
	UARTsInit();
	UARTShowVersion();
	nRF24L01P.transmissionTimeout = -1;
	nRF24L01P.mode = NRF24L01P_MODE_UNKNOWN;
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, LED_PORT, LED_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, LED_PORT, LED_PIN);

	uint32_t load = Chip_Clock_GetSystemClockRate() / 1000 - 1;
	if (load > 0xFFFFFF) {
		load = 0xFFFFFF;
	}
	SysTick->LOAD = load;
	SysTick->CTRL |= 0x7;	//enable the Systick
	while (1) {
		int count = RingBuffer_GetCount(&uart0RX);
		if (count) {
			if (throughMode) {
				if (count > 64) {
					count = 64;
				}
				RingBuffer_PopMult(&uart0RX, buffer, 64);
				if (RFpushDataToBuffer(buffer, count)) {
					//TODO: handle this data that couldn't fit
				}
			} else {
				UART0ParseNewChar();
			}
		}
#if 1
		if (throughMode == 2) {
			++eventCounter;
			fakeEvent[0] = (eventCounter & 127) | 0x80;
			fakeEvent[1] = eventCounter >> 8;
			RFpushDataToBuffer(fakeEvent, 2);
		}
#endif
		if (toggleLed0) {
			toggleLed0 = 0;
			Chip_GPIO_SetPinToggle(LPC_GPIO, LED_PORT, LED_PIN);
		}
		if (throughMode) {
			RFiterate();
		}
	}
	return 0;
}
