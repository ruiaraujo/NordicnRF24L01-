/*
 * utils.c
 *
 *  Created on: Jun 4, 2014
 *      Author: raraujo
 */

#include "chip.h"

static inline void busyWaiting(uint32_t count, uint32_t divider) {
	int32_t curr = (int32_t) Chip_RIT_GetCounter(LPC_RITIMER);
	int32_t final = curr
			+ (((SystemCoreClock / Chip_Clock_GetPCLKDiv(SYSCTL_PCLK_RIT))
					/ divider) * count);

	if (final == curr)
		return;

	if ((final < 0) && (curr > 0)) {
		while (Chip_RIT_GetCounter(LPC_RITIMER) < (uint32_t) final) {
		}
	} else {
		while ((int32_t) Chip_RIT_GetCounter(LPC_RITIMER) < final) {
		}
	}
}

void timerDelayUs(uint32_t timeUs) {
	busyWaiting(timeUs, 1000000);
}
void timerDelayMs(uint32_t timeMs) {
	busyWaiting(timeMs, 1000);
}

