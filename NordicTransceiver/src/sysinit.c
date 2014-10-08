/*
 * @brief Common SystemInit function for LPC17xx/40xx chips
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
#include "chip.h"
#include "pinout.h"
const uint32_t OscRateIn = 12000000;
const uint32_t RTCOscRateIn = 32768;

static const PINMUX_GRP_T pinmuxing[] = {
		//UART0
		{ UART0_TX_PORT, UART0_TX_PIN, IOCON_MODE_INACT | IOCON_FUNC1 }, /* TXD0 */
		{ UART0_RX_PORT, UART0_RX_PIN, IOCON_MODE_INACT | IOCON_FUNC1 }, /* RXD0 */

		{ MOSI_PORT, MOSI_PIN, IOCON_MODE_INACT | IOCON_FUNC2 }, /* MOSI */
		{ MISO_PORT, MISO_PIN, IOCON_MODE_INACT | IOCON_FUNC2 }, /* MISO */
		{ SCK_PORT, SCK_PIN, IOCON_MODE_INACT | IOCON_FUNC2 }, /* SCK */
		{ CSN_PORT, CSN_PIN, IOCON_MODE_INACT | IOCON_FUNC0 }, /* SSEL */

		{ NORDIC_CE_PORT, NORDIC_CE_PIN, IOCON_MODE_INACT | IOCON_FUNC0 }, /* CE pin for the Nordic */
		{ NORDIC_IRQ_PORT, NORDIC_IRQ_PIN, IOCON_MODE_INACT | IOCON_FUNC0 }, /* IRQ pin for the Nordic */

		{ LED_PORT, LED_PIN, IOCON_MODE_INACT | IOCON_FUNC0 }, /* Led 0 */

};

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set up and initialize hardware prior to call to main */
void SystemInit(void) {
	unsigned int *pSCB_VTOR = (unsigned int *) 0xE000ED08;

#if defined(__IAR_SYSTEMS_ICC__)
	extern void *__vector_table;

	*pSCB_VTOR = (unsigned int) &__vector_table;
#elif defined(__CODE_RED)
	extern void *g_pfnVectors;

	*pSCB_VTOR = (unsigned int) &g_pfnVectors;
#elif defined(__ARMCC_VERSION)
	extern void *__Vectors;

	*pSCB_VTOR = (unsigned int) &__Vectors;
#endif

#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1
	fpuInit();
#endif

	/* Chip specific SystemInit */
	Chip_SystemInit();
	SystemCoreClockUpdate();
	Chip_RIT_Init(LPC_RITIMER);
	Chip_GPIO_Init(LPC_GPIO);
	Chip_IOCON_Init(LPC_IOCON);
	Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing,
			sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));
	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_UART0, SYSCTL_CLKDIV_1);
	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_SSP1, SYSCTL_CLKDIV_1);
}
