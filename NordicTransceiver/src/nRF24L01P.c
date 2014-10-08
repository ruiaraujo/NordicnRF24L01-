/**
 * @file nRF24L01P.cpp
 *
 * @author Owen Edwards
 * 
 * @section LICENSE
 *
 * Copyright (c) 2010 Owen Edwards
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * @section DESCRIPTION
 *
 * nRF24L01+ Single Chip 2.4GHz Transceiver from Nordic Semiconductor.
 *
 * Datasheet:
 *
 * http://www.nordicsemi.no/files/Product/data_sheet/nRF24L01P_Product_Specification_1_0.pdf
 */

/**
 * Includes
 */
#include <string.h>
#include "nRF24L01P.h"
#include "chip.h"
#include "utils.h"
#include "xprintf.h"
#include "pinout.h"

#define NRF24L01P_REG_ADDRESS_MASK          0x1f

// CONFIG register:
#define NRF24L01P_CONFIG_PRIM_RX        (1<<0)
#define NRF24L01P_CONFIG_PWR_UP         (1<<1)
#define NRF24L01P_CONFIG_CRC0           (1<<2)
#define NRF24L01P_CONFIG_EN_CRC         (1<<3)
#define NRF24L01P_CONFIG_MASK_MAX_RT    (1<<4)
#define NRF24L01P_CONFIG_MASK_TX_DS     (1<<5)
#define NRF24L01P_CONFIG_MASK_RX_DR     (1<<6)

#define NRF24L01P_CONFIG_CRC_MASK       (NRF24L01P_CONFIG_EN_CRC|NRF24L01P_CONFIG_CRC0)
#define NRF24L01P_CONFIG_CRC_NONE       (0)
#define NRF24L01P_CONFIG_CRC_8BIT       (NRF24L01P_CONFIG_EN_CRC)
#define NRF24L01P_CONFIG_CRC_16BIT      (NRF24L01P_CONFIG_EN_CRC|NRF24L01P_CONFIG_CRC0)

// EN_AA register:
#define NRF24L01P_EN_AA_NONE            0

// EN_RXADDR register:
#define NRF24L01P_EN_RXADDR_NONE        0

// SETUP_AW register:
#define NRF24L01P_SETUP_AW_AW_MASK      (0x3<<0)
#define NRF24L01P_SETUP_AW_AW_3BYTE     (0x1<<0)
#define NRF24L01P_SETUP_AW_AW_4BYTE     (0x2<<0)
#define NRF24L01P_SETUP_AW_AW_5BYTE     (0x3<<0)

// SETUP_RETR register:
#define NRF24L01P_SETUP_RETR_NONE       	0
#define NRF24L01P_SETUP_RETR_MASK     		0xF
#define NRF24L01P_SETUP_RETR_ARD_DIVIDER	250

// RF_SETUP register:
#define NRF24L01P_RF_SETUP_RF_PWR_MASK          (0x3<<1)
#define NRF24L01P_RF_SETUP_RF_PWR_0DBM          (0x3<<1)
#define NRF24L01P_RF_SETUP_RF_PWR_MINUS_6DBM    (0x2<<1)
#define NRF24L01P_RF_SETUP_RF_PWR_MINUS_12DBM   (0x1<<1)
#define NRF24L01P_RF_SETUP_RF_PWR_MINUS_18DBM   (0x0<<1)

#define NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT       (1 << 3)
#define NRF24L01P_RF_SETUP_RF_DR_LOW_BIT        (1 << 5)
#define NRF24L01P_RF_SETUP_RF_DR_MASK           (NRF24L01P_RF_SETUP_RF_DR_LOW_BIT|NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT)
#define NRF24L01P_RF_SETUP_RF_DR_250KBPS        (NRF24L01P_RF_SETUP_RF_DR_LOW_BIT)
#define NRF24L01P_RF_SETUP_RF_DR_1MBPS          (0)
#define NRF24L01P_RF_SETUP_RF_DR_2MBPS          (NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT)

// RX_PW_P0..RX_PW_P5 registers:
#define NRF24L01P_RX_PW_Px_MASK         0x3F

//FIFO status register
#define NRF24L01P_RX_EMPTY        	(1<<0)

//DYNPD register
#define NRF24L01P_DPL_P0        	(1<<0)
#define NRF24L01P_DPL_P1        	(1<<1)
#define NRF24L01P_DPL_P2        	(1<<2)
#define NRF24L01P_DPL_P3        	(1<<3)
#define NRF24L01P_DPL_P4        	(1<<4)
#define NRF24L01P_DPL_P5        	(1<<5)

//FEATURE register
#define NRF24L01P_EN_DYN_ACK        	(1<<0)
#define NRF24L01P_EN_ACK_PAY        	(1<<1)
#define NRF24L01P_EN_DPL      		  	(1<<2)

#define NRF24L01P_TIMING_Tundef2pd_us     100000   // 100mS
#define NRF24L01P_TIMING_Tstby2a_us          130   // 130uS
#define NRF24L01P_TIMING_Thce_us              10   //  10uS
#define NRF24L01P_TIMING_Tpd2stby_us        4500   // 4.5mS worst case
#define NRF24L01P_TIMING_Tpece2csn_us          4   //   4uS

#define NRF24L01P_TX_TIMEOUT		    10   // 10mS
#define NRF24L01P_TX_TIMEOUT_ACK		10   // 10mS

struct NRF24L01P nRF24L01P;
static Chip_SSP_DATA_SETUP_T spi;

#define TX_BUFFER_SIZE_BITS		(12)
#define TX_BUFFER_SIZE			(1<<TX_BUFFER_SIZE_BITS)

static RINGBUFF_T nordicTxBuffer;
static char nordictxBufferArray[TX_BUFFER_SIZE];

static uint8_t spiBufferTx[NRF24L01P_TX_FIFO_SIZE + 1];
static uint8_t spiBufferRx[NRF24L01P_RX_FIFO_SIZE + 1];

void writeToSPI(int length) {
	spi.length = length;
	spi.rx_cnt = 0;
	spi.tx_cnt = 0;

	Chip_GPIO_SetPinOutLow(LPC_GPIO, CSN_PORT, CSN_PIN);
	Chip_SSP_RWFrames_Blocking(LPC_SSP1, &spi);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, CSN_PORT, CSN_PIN);
}

/**
 * Methods
 */
void nRF24L01PInit() {

	Chip_GPIO_SetPinOutHigh(LPC_GPIO, CSN_PORT, CSN_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, CSN_PORT, CSN_PIN);
	Chip_GPIO_SetPinOutLow(LPC_GPIO, NORDIC_CE_PORT, NORDIC_CE_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, NORDIC_CE_PORT, NORDIC_CE_PIN);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, NORDIC_IRQ_PORT, NORDIC_IRQ_PIN);
	nRF24L01P.mode = NRF24L01P_MODE_UNKNOWN;
	nRF24L01P.transmissionTimeout = -1;
	nRF24L01P.ACKPayloadsinTXFIFO = 0;
	RFdisable();
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SSP1);
	Chip_SSP_Set_Mode(LPC_SSP1, SSP_MODE_MASTER);
	Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL0); // 8-bit, ClockPhase = 0, ClockPolarity = 0
	Chip_SSP_SetBitRate(LPC_SSP1, NRF24L01P_SPI_MAX_DATA_RATE / 5); // 2Mbit, 1/5th the maximum transfer rate for the SPI bus
	Chip_SSP_Enable(LPC_SSP1);

	RingBuffer_Init(&nordicTxBuffer, nordictxBufferArray, 1, TX_BUFFER_SIZE);
	memset(&spi, 0, sizeof(spi));
	spi.tx_data = spiBufferTx;
	spi.rx_data = spiBufferRx;

	timerDelayUs(NRF24L01P_TIMING_Tundef2pd_us);    // Wait for Power-on reset

	RFsetRegister(NRF24L01P_REG_CONFIG, 0); // Power Down
	RFsetRegister(NRF24L01P_REG_STATUS, NRF24L01P_STATUS_MAX_RT | NRF24L01P_STATUS_TX_DS | NRF24L01P_STATUS_RX_DR);  // Clear any pending interrupts
	RFsetRegister(NRF24L01P_REG_FEATURE, NRF24L01P_EN_DYN_ACK | NRF24L01P_EN_ACK_PAY | NRF24L01P_EN_DPL); //Enabled no ack packages
	RFsetRegister(NRF24L01P_REG_DYNPD, NRF24L01P_DPL_P0);
	//
	// Setup default configuration
	//
	RFdisableAllRxPipes();
	RFsetFrequency(DEFAULTNRF24L01P_RF_FREQUENCY);
	RFsetAirDataRate(DEFAULTNRF24L01P_DATARATE);
	RFsetRfOutputPower(DEFAULTNRF24L01P_TX_PWR);
	RFsetCrcWidth(DEFAULTNRF24L01P_CRC);
	RFsetTxAddress(DEFAULTNRF24L01P_ADDRESS, DEFAULTNRF24L01P_ADDRESS_WIDTH);
	RFsetRxAddress(DEFAULTNRF24L01P_ADDRESS, DEFAULTNRF24L01P_ADDRESS_WIDTH, NRF24L01P_PIPE_P0);
	RFenableAutoRetransmit(250, 3);
	RFdisableAutoAcknowledge();
	RFenableAutoAcknowledge(NRF24L01P_PIPE_P0);
	RFsetTransferSize(DEFAULTNRF24L01P_TRANSFER_SIZE, NRF24L01P_PIPE_P0);
	nRF24L01P.mode = NRF24L01P_MODE_POWER_DOWN;
	RFsetReceiveMode();
	RFenable();
}

void RFpowerUp(void) {
	int config = RFgetRegister(NRF24L01P_REG_CONFIG);
	config |= NRF24L01P_CONFIG_PWR_UP;
	RFsetRegister(NRF24L01P_REG_CONFIG, config);
	// Wait until the nRF24L01+ powers up
	timerDelayUs( NRF24L01P_TIMING_Tpd2stby_us);
	nRF24L01P.mode = NRF24L01P_MODE_STANDBY;
}

void RFpowerDown(void) {
	int config = RFgetRegister(NRF24L01P_REG_CONFIG);
	config &= ~NRF24L01P_CONFIG_PWR_UP;
	RFsetRegister(NRF24L01P_REG_CONFIG, config);
	// Wait until the nRF24L01+ powers down
	timerDelayUs( NRF24L01P_TIMING_Tpd2stby_us); // This *may* not be necessary (no timing is shown in the Datasheet), but just to be safe
	nRF24L01P.mode = NRF24L01P_MODE_POWER_DOWN;
}

void RFsetReceiveMode(void) {
	if (NRF24L01P_MODE_POWER_DOWN == nRF24L01P.mode) {
		RFpowerUp();
	}
	int config = RFgetRegister(NRF24L01P_REG_CONFIG);
	config |= NRF24L01P_CONFIG_PRIM_RX;
	RFsetRegister(NRF24L01P_REG_CONFIG, config);
	nRF24L01P.mode = NRF24L01P_MODE_RX;
}

void RFsetTransmitMode(void) {
	if (NRF24L01P_MODE_POWER_DOWN == nRF24L01P.mode) {
		RFpowerUp();
	}
	int config = RFgetRegister(NRF24L01P_REG_CONFIG);
	config &= ~NRF24L01P_CONFIG_PRIM_RX;
	RFsetRegister(NRF24L01P_REG_CONFIG, config);
	nRF24L01P.mode = NRF24L01P_MODE_TX;
}

void RFenable(void) {
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, NORDIC_CE_PORT, NORDIC_CE_PIN);
	timerDelayUs( NRF24L01P_TIMING_Tpece2csn_us);
}

void RFdisable(void) {
	Chip_GPIO_SetPinOutLow(LPC_GPIO, NORDIC_CE_PORT, NORDIC_CE_PIN);
}

void RFsetFrequency(int frequency) {
	if ((frequency < NRF24L01P_MIN_RF_FREQUENCY) || (frequency > NRF24L01P_MAX_RF_FREQUENCY)) {

		xprintf("nRF24L01P: Invalid RF Frequency setting %d\r\n", frequency);
		return;
	}
	int channel = (frequency - NRF24L01P_MIN_RF_FREQUENCY) & 0x7F;
	RFsetRegister(NRF24L01P_REG_RF_CH, channel);
}

int RFgetFrequency(void) {
	int channel = RFgetRegister(NRF24L01P_REG_RF_CH) & 0x7F;
	return (channel + NRF24L01P_MIN_RF_FREQUENCY);
}

void RFsetRfOutputPower(int power) {
	int rfSetup = RFgetRegister(NRF24L01P_REG_RF_SETUP) & ~NRF24L01P_RF_SETUP_RF_PWR_MASK;
	switch (power) {

	case NRF24L01P_TX_PWR_ZERO_DB:
		rfSetup |= NRF24L01P_RF_SETUP_RF_PWR_0DBM;
		break;

	case NRF24L01P_TX_PWR_MINUS_6_DB:
		rfSetup |= NRF24L01P_RF_SETUP_RF_PWR_MINUS_6DBM;
		break;

	case NRF24L01P_TX_PWR_MINUS_12_DB:
		rfSetup |= NRF24L01P_RF_SETUP_RF_PWR_MINUS_12DBM;
		break;

	case NRF24L01P_TX_PWR_MINUS_18_DB:
		rfSetup |= NRF24L01P_RF_SETUP_RF_PWR_MINUS_18DBM;
		break;

	default:
		xprintf("nRF24L01P: Invalid RF Output Power setting %d\r\n", power);
		return;

	}
	RFsetRegister(NRF24L01P_REG_RF_SETUP, rfSetup);
}

int RFgetRfOutputPower(void) {
	int rfPwr = RFgetRegister(NRF24L01P_REG_RF_SETUP) & NRF24L01P_RF_SETUP_RF_PWR_MASK;
	switch (rfPwr) {

	case NRF24L01P_RF_SETUP_RF_PWR_0DBM:
		return NRF24L01P_TX_PWR_ZERO_DB;

	case NRF24L01P_RF_SETUP_RF_PWR_MINUS_6DBM:
		return NRF24L01P_TX_PWR_MINUS_6_DB;

	case NRF24L01P_RF_SETUP_RF_PWR_MINUS_12DBM:
		return NRF24L01P_TX_PWR_MINUS_12_DB;

	case NRF24L01P_RF_SETUP_RF_PWR_MINUS_18DBM:
		return NRF24L01P_TX_PWR_MINUS_18_DB;

	default:
		xprintf("nRF24L01P: Unknown RF Output Power value %d\r\n", rfPwr);
		return 0;

	}
}

void RFsetAirDataRate(int rate) {
	int rfSetup = RFgetRegister(NRF24L01P_REG_RF_SETUP) & ~NRF24L01P_RF_SETUP_RF_DR_MASK;
	switch (rate) {
	case NRF24L01P_DATARATE_250_KBPS:
		rfSetup |= NRF24L01P_RF_SETUP_RF_DR_250KBPS;
		break;

	case NRF24L01P_DATARATE_1_MBPS:
		rfSetup |= NRF24L01P_RF_SETUP_RF_DR_1MBPS;
		break;

	case NRF24L01P_DATARATE_2_MBPS:
		rfSetup |= NRF24L01P_RF_SETUP_RF_DR_2MBPS;
		break;

	default:
		xprintf("nRF24L01P: Invalid Air Data Rate setting %d\r\n", rate);
		return;

	}
	RFsetRegister(NRF24L01P_REG_RF_SETUP, rfSetup);
}

int RFgetAirDataRate(void) {
	int rfDataRate = RFgetRegister(NRF24L01P_REG_RF_SETUP) & NRF24L01P_RF_SETUP_RF_DR_MASK;
	switch (rfDataRate) {
	case NRF24L01P_RF_SETUP_RF_DR_250KBPS:
		return NRF24L01P_DATARATE_250_KBPS;
	case NRF24L01P_RF_SETUP_RF_DR_1MBPS:
		return NRF24L01P_DATARATE_1_MBPS;
	case NRF24L01P_RF_SETUP_RF_DR_2MBPS:
		return NRF24L01P_DATARATE_2_MBPS;
	default:
		xprintf("nRF24L01P: Unknown Air Data Rate value %d\r\n", rfDataRate);
		return 0;
	}
}

void RFsetCrcWidth(int width) {
	int config = RFgetRegister(NRF24L01P_REG_CONFIG) & ~NRF24L01P_CONFIG_CRC_MASK;
	switch (width) {

	case NRF24L01P_CRC_NONE:
		config |= NRF24L01P_CONFIG_CRC_NONE;
		break;
	case NRF24L01P_CRC_8_BIT:
		config |= NRF24L01P_CONFIG_CRC_8BIT;
		break;
	case NRF24L01P_CRC_16_BIT:
		config |= NRF24L01P_CONFIG_CRC_16BIT;
		break;
	default:
		xprintf("nRF24L01P: Invalid CRC Width setting %d\r\n", width);
		return;
	}
	RFsetRegister(NRF24L01P_REG_CONFIG, config);
}

int RFgetCrcWidth(void) {
	int crcWidth = RFgetRegister( NRF24L01P_REG_CONFIG) & NRF24L01P_CONFIG_CRC_MASK;
	switch (crcWidth) {

	case NRF24L01P_CONFIG_CRC_NONE:
		return NRF24L01P_CRC_NONE;

	case NRF24L01P_CONFIG_CRC_8BIT:
		return NRF24L01P_CRC_8_BIT;

	case NRF24L01P_CONFIG_CRC_16BIT:
		return NRF24L01P_CRC_16_BIT;

	default:
		xprintf("nRF24L01P: Unknown CRC Width value %d\r\n", crcWidth);
		return 0;

	}
}

void RFsetTransferSize(int size, int pipe) {
	if ((pipe < NRF24L01P_PIPE_P0) || (pipe > NRF24L01P_PIPE_P5)) {
		xprintf("nRF24L01P: Invalid Transfer Size pipe number %d\r\n", pipe);
		return;
	}
	if ((size < 0) || (size > NRF24L01P_RX_FIFO_SIZE)) {
		xprintf("nRF24L01P: Invalid Transfer Size setting %d\r\n", size);
		return;
	}
	int rxPwPxRegister = NRF24L01P_REG_RX_PW_P0 + (pipe - NRF24L01P_PIPE_P0);
	RFsetRegister(rxPwPxRegister, (size & NRF24L01P_RX_PW_Px_MASK));
}

int RFgetTransferSize(int pipe) {
	if ((pipe < NRF24L01P_PIPE_P0) || (pipe > NRF24L01P_PIPE_P5)) {
		xprintf("nRF24L01P: Invalid Transfer Size pipe number %d\r\n", pipe);
		return 0;
	}
	int rxPwPxRegister = NRF24L01P_REG_RX_PW_P0 + (pipe - NRF24L01P_PIPE_P0);
	int size = RFgetRegister(rxPwPxRegister);
	return (size & NRF24L01P_RX_PW_Px_MASK);
}

void RFdisableAllRxPipes(void) {
	RFsetRegister(NRF24L01P_REG_EN_RXADDR, NRF24L01P_EN_RXADDR_NONE);
}

void RFdisableAutoAcknowledge(void) {
	RFsetRegister(NRF24L01P_REG_EN_AA, NRF24L01P_EN_AA_NONE);
}

void RFenableAutoAcknowledge(int pipe) {
	if ((pipe < NRF24L01P_PIPE_P0) || (pipe > NRF24L01P_PIPE_P5)) {
		xprintf("nRF24L01P: Invalid Enable AutoAcknowledge pipe number %d\r\n", pipe);
		return;
	}
	int enAA = RFgetRegister(NRF24L01P_REG_EN_AA);
	enAA |= (1 << (pipe - NRF24L01P_PIPE_P0));
	RFsetRegister(NRF24L01P_REG_EN_AA, enAA);
}

void RFenableACKTX(bool enable) {
	if (enable) {
		nRF24L01P.mode |= NRF24L01P_MODE_TX_NO_ACK;
	} else {
		nRF24L01P.mode &= ~NRF24L01P_MODE_TX_NO_ACK;
	}
}
void RFdisableAutoRetransmit(void) {
	RFsetRegister(NRF24L01P_REG_SETUP_RETR, NRF24L01P_SETUP_RETR_NONE);
}

void RFenableAutoRetransmit(int delay, int count) {
	int ard = (delay / NRF24L01P_SETUP_RETR_ARD_DIVIDER) - 1;
	if (ard < 0) {
		ard = 0;
	}
	if (ard > NRF24L01P_SETUP_RETR_MASK) {
		ard = NRF24L01P_SETUP_RETR_MASK;
	}
	RFsetRegister(NRF24L01P_REG_SETUP_RETR, (ard << 4) | (count & NRF24L01P_SETUP_RETR_MASK));
}

void RFsetRxAddress(unsigned long long address, int width, int pipe) {
	if ((pipe < NRF24L01P_PIPE_P0) || (pipe > NRF24L01P_PIPE_P5)) {
		xprintf("nRF24L01P: Invalid setRxAddress pipe number %d\r\n", pipe);
		return;
	}
	if ((pipe == NRF24L01P_PIPE_P0) || (pipe == NRF24L01P_PIPE_P1)) {
		int setupAw = RFgetRegister(NRF24L01P_REG_SETUP_AW) & ~NRF24L01P_SETUP_AW_AW_MASK;
		switch (width) {
		case 3:
			setupAw |= NRF24L01P_SETUP_AW_AW_3BYTE;
			break;

		case 4:
			setupAw |= NRF24L01P_SETUP_AW_AW_4BYTE;
			break;

		case 5:
			setupAw |= NRF24L01P_SETUP_AW_AW_5BYTE;
			break;

		default:
			xprintf("nRF24L01P: Invalid setRxAddress width setting %d\r\n", width);
			return;
		}
		RFsetRegister(NRF24L01P_REG_SETUP_AW, setupAw);
	} else {
		width = 1;
	}
	int rxAddrPxRegister = NRF24L01P_REG_RX_ADDR_P0 + (pipe - NRF24L01P_PIPE_P0);
	int i = 0;
	spiBufferTx[i++] = (NRF24L01P_SPI_CMD_WR_REG | (rxAddrPxRegister & NRF24L01P_REG_ADDRESS_MASK));
	while (width-- > 0) {
		// LSByte first
		spiBufferTx[i++] = address & 0xFF;
		address >>= 8;
	}
	writeToSPI(i);
	int enRxAddr = RFgetRegister(NRF24L01P_REG_EN_RXADDR);
	enRxAddr |= (1 << (pipe - NRF24L01P_PIPE_P0));
	RFsetRegister(NRF24L01P_REG_EN_RXADDR, enRxAddr);
}

void RFsetTxAddress(unsigned long long address, int width) {
	int setupAw = RFgetRegister(NRF24L01P_REG_SETUP_AW) & ~NRF24L01P_SETUP_AW_AW_MASK;
	switch (width) {
	case 3:
		setupAw |= NRF24L01P_SETUP_AW_AW_3BYTE;
		break;

	case 4:
		setupAw |= NRF24L01P_SETUP_AW_AW_4BYTE;
		break;

	case 5:
		setupAw |= NRF24L01P_SETUP_AW_AW_5BYTE;
		break;

	default:
		xprintf("nRF24L01P: Invalid setTxAddress width setting %d\r\n", width);
		return;
	}
	RFsetRegister(NRF24L01P_REG_SETUP_AW, setupAw);
	int i = 0;
	spiBufferTx[i++] = (NRF24L01P_SPI_CMD_WR_REG | (NRF24L01P_REG_TX_ADDR & NRF24L01P_REG_ADDRESS_MASK));
	while (width-- > 0) {
		// LSByte first
		spiBufferTx[i++] = address & 0xFF;
		address >>= 8;
	}
	writeToSPI(i);
}

unsigned long long RFgetRxAddress(int pipe) {
	if ((pipe < NRF24L01P_PIPE_P0) || (pipe > NRF24L01P_PIPE_P5)) {
		xprintf("nRF24L01P: Invalid setRxAddress pipe number %d\r\n", pipe);
		return 0;
	}
	int width;
	if ((pipe == NRF24L01P_PIPE_P0) || (pipe == NRF24L01P_PIPE_P1)) {
		int setupAw = RFgetRegister(NRF24L01P_REG_SETUP_AW) & NRF24L01P_SETUP_AW_AW_MASK;
		switch (setupAw) {

		case NRF24L01P_SETUP_AW_AW_3BYTE:
			width = 3;
			break;

		case NRF24L01P_SETUP_AW_AW_4BYTE:
			width = 4;
			break;

		case NRF24L01P_SETUP_AW_AW_5BYTE:
			width = 5;
			break;

		default:
			xprintf("nRF24L01P: Unknown getRxAddress width value %d\r\n", setupAw);
			return 0;
		}
	} else {
		width = 1;
	}

	int rxAddrPxRegister = NRF24L01P_REG_RX_ADDR_P0 + (pipe - NRF24L01P_PIPE_P0);
	spiBufferTx[0] = (NRF24L01P_SPI_CMD_RD_REG | (rxAddrPxRegister & NRF24L01P_REG_ADDRESS_MASK));
	for (int i = 0; i < width; i++) {
		spiBufferTx[i + 1] = NRF24L01P_SPI_CMD_NOP;
	}
	writeToSPI(width + 1);
	unsigned long long address = 0;
	for (int i = 0; i < width; i++) {
		// LSByte first
		address |= (((unsigned long long) (spiBufferRx[i + 1] & 0xFF)) << (i * 8));

	}
	if (!((pipe == NRF24L01P_PIPE_P0) || (pipe == NRF24L01P_PIPE_P1))) {
		address |= (RFgetRxAddress(NRF24L01P_PIPE_P1) & ~((unsigned long long) 0xFF));
	}
	return address;

}

unsigned long long RFgetTxAddress(void) {
	int setupAw = RFgetRegister(NRF24L01P_REG_SETUP_AW) & NRF24L01P_SETUP_AW_AW_MASK;
	int width;
	switch (setupAw) {
	case NRF24L01P_SETUP_AW_AW_3BYTE:
		width = 3;
		break;
	case NRF24L01P_SETUP_AW_AW_4BYTE:
		width = 4;
		break;
	case NRF24L01P_SETUP_AW_AW_5BYTE:
		width = 5;
		break;
	default:
		xprintf("nRF24L01P: Unknown getTxAddress width value %d\r\n", setupAw);
		return 0;

	}

	spiBufferTx[0] = (NRF24L01P_SPI_CMD_RD_REG | (NRF24L01P_REG_TX_ADDR & NRF24L01P_REG_ADDRESS_MASK));
	for (int i = 0; i < width; i++) {
		spiBufferTx[i + 1] = NRF24L01P_SPI_CMD_NOP;
	}
	writeToSPI(width + 1);
	unsigned long long address = 0;
	for (int i = 0; i < width; i++) {
		// LSByte first
		address |= (((unsigned long long) (spiBufferRx[i + 1] & 0xFF)) << (i * 8));

	}
	return address;
}

bool RFisRXFIFOEmpty() {
	int fifoStatus = RFgetRegister(NRF24L01P_REG_FIFO_STATUS);
	return (fifoStatus & NRF24L01P_RX_EMPTY) == 0x01;
}

bool RFreadable(int pipe) {
	if ((pipe < NRF24L01P_PIPE_P0) || (pipe > NRF24L01P_PIPE_P5)) {
		xprintf("nRF24L01P: Invalid readable pipe number %d\r\n", pipe);
		return false;

	}
	int status = RFgetStatusRegister();
	return ((status & NRF24L01P_STATUS_RX_DR) && (((status & NRF24L01P_STATUS_RX_P_NO) >> 1) == (pipe & 0x7)));
}

void RFsetNormalMode() {
	nRF24L01P.mode &= ~NRF24L01P_MODE_TRANSMITTING;
}

void RFsetTransmittingMode() {
	nRF24L01P.mode |= NRF24L01P_MODE_TRANSMITTING;
}

void RFretransmistLastPayload() {
	spiBufferTx[0] = NRF24L01P_SPI_CMD_REUSE_TX_PL;
	writeToSPI(1);
}

void RFFlushTXFIFO() {
	spiBufferTx[0] = NRF24L01P_SPI_CMD_FLUSH_TX;
	writeToSPI(1);
}

void RFFlushRXFIFO() {
	spiBufferTx[0] = NRF24L01P_SPI_CMD_FLUSH_RX;
	writeToSPI(1);
}

int RFgetRXPayloadSize() {
	spiBufferTx[0] = NRF24L01P_SPI_CMD_R_RX_PL_WID;
	spiBufferTx[1] = NRF24L01P_SPI_CMD_NOP;
	writeToSPI(2);
	int rxPayloadWidth = spiBufferRx[1];
	if ((rxPayloadWidth < 0) || (rxPayloadWidth > NRF24L01P_RX_FIFO_SIZE)) {
		return -1;
	}
	return rxPayloadWidth;
}

void RFRXFIFOread(char *data, int count) {
	if (count <= 0 || count > NRF24L01P_RX_FIFO_SIZE) {
		return;
	}
	spiBufferTx[0] = NRF24L01P_SPI_CMD_RD_RX_PAYLOAD;
	for (int i = 0; i < count; i++) {
		spiBufferTx[i + 1] = NRF24L01P_SPI_CMD_NOP;
	}
	writeToSPI(count + 1);
	for (int i = 0; i < count; i++) {
		*data++ = spiBufferRx[i + 1];
	}
}

int RFpushDataToBuffer(unsigned char *data, int count) {
	int ret = RingBuffer_InsertMult(&nordicTxBuffer, data, count);
	//If the buffer has enough data to be sent, send it right away
	if (RingBuffer_GetCount(&nordicTxBuffer) >= NRF24L01P_TX_FIFO_SIZE && (nRF24L01P.mode & NRF24L01P_MODE_TRANSMITTING) == 0) {
		RFmoveBufferToTransmission(NRF24L01P_PIPE_P0);
		RFsetTransmittingMode();
	}
	/* Add additional data to transmit ring buffer if possible */
	if (ret != count) {
		ret += RingBuffer_InsertMult(&nordicTxBuffer, (data + ret), (count - ret));
	}
	if (RingBuffer_GetCount(&nordicTxBuffer) > 0 && (nRF24L01P.mode & NRF24L01P_MODE_TRANSMITTING) == 0) {
		//If there is data to be sent, start a timeout for transmission
		//If there was a previous transmission no need to do this
		if (nRF24L01P.mode & NRF24L01P_MODE_TX) {
			nRF24L01P.transmissionTimeout = NRF24L01P_TX_TIMEOUT;
		} else if (nRF24L01P.mode & NRF24L01P_MODE_RX) {
			nRF24L01P.transmissionTimeout = NRF24L01P_TX_TIMEOUT_ACK;
		}
	}
	return ret;
}

bool RFisTXBufferEmpty() {
	return RingBuffer_GetCount(&nordicTxBuffer) == 0;
}

void RFmoveBufferToTransmission(int pipe) {
	int count = RingBuffer_GetCount(&nordicTxBuffer);
	if (nRF24L01P.mode & NRF24L01P_MODE_TX) {
		if (count > NRF24L01P_TX_FIFO_SIZE) {
			count = NRF24L01P_TX_FIFO_SIZE;
		}
		if ( nRF24L01P.mode & NRF24L01P_MODE_TX_NO_ACK ){
			spiBufferTx[0] = NRF24L01P_SPI_CMD_W_TX_PYLD_NO_ACK;
		}else{
			spiBufferTx[0] = NRF24L01P_SPI_CMD_WR_TX_PAYLOAD;
		}
	} else {
		if ((pipe < NRF24L01P_PIPE_P0) || (pipe > NRF24L01P_PIPE_P5)) {
			xprintf("nRF24L01P: Invalid read pipe number %d\r\n", pipe);
			return;
		}
		if (nRF24L01P.ACKPayloadsinTXFIFO >= NRF24L01P_TX_FIFO_COUNT) {
			return; //We have pushed the maximum amount of payload to the Nordic in RX mode
		}
		if (count > NRF24L01P_TX_ACK_FIFO_SIZE) {
			count = NRF24L01P_TX_ACK_FIFO_SIZE;
		}
		spiBufferTx[0] = NRF24L01P_SPI_CMD_W_ACK_PAYLOAD | (pipe & 0x7);
		nRF24L01P.ACKPayloadsinTXFIFO++;
	}
	RingBuffer_PopMult(&nordicTxBuffer, spiBufferTx + 1, count);
	RFsetTransmittingMode();
	writeToSPI(count + 1);
}

int RFsimple_read(int pipe, char *data, int count) {
	if ((pipe < NRF24L01P_PIPE_P0) || (pipe > NRF24L01P_PIPE_P5)) {
		xprintf("nRF24L01P: Invalid read pipe number %d\r\n", pipe);
		return -1;
	}

	if (count <= 0)
		return 0;

	if (count > NRF24L01P_RX_FIFO_SIZE)
		count = NRF24L01P_RX_FIFO_SIZE;

	if (RFreadable(pipe)) {
		int rxPayloadWidth = RFgetRXPayloadSize();

		if (rxPayloadWidth < 0) {
			// Received payload error: need to flush the FIFO
			RFFlushRXFIFO();
			// At this point, we should retry the reception,
			//  but for now we'll just fall through...
			//

		} else {
			if (rxPayloadWidth < count)
				count = rxPayloadWidth;
			RFRXFIFOread(data, count);
			// Clear the Status bit
			RFsetRegister(NRF24L01P_REG_STATUS, NRF24L01P_STATUS_RX_DR);
			return count;
		}
	} else {
		//
		// What should we do if there is no 'readable' data?
		//  We could wait for data to arrive, but for now, we'll
		//  just return with no data.
		//
		return 0;
	}

	//
	// We get here because an error condition occured;
	//  We could wait for data to arrive, but for now, we'll
	//  just return with no data.
	//
	return -1;

}

void RFsetRegister(int regAddress, int regData) {
	//
	// Save the CE state
	//
	int originalCe = Chip_GPIO_ReadPortBit(LPC_GPIO, NORDIC_CE_PORT, NORDIC_CE_PIN);
	RFdisable();
	spiBufferTx[0] = (NRF24L01P_SPI_CMD_WR_REG | (regAddress & NRF24L01P_REG_ADDRESS_MASK));
	spiBufferTx[1] = regData & 0xFF;
	writeToSPI(2);

	if (originalCe) {
		RFenable();
	} else {
		RFdisable();
	}
	timerDelayUs( NRF24L01P_TIMING_Tpece2csn_us);
}

int RFgetRegister(int regAddress) {
	spiBufferTx[0] = (NRF24L01P_SPI_CMD_RD_REG | (regAddress & NRF24L01P_REG_ADDRESS_MASK));
	spiBufferTx[1] = NRF24L01P_SPI_CMD_NOP;
	writeToSPI(2);
	return spiBufferRx[1];

}

int RFgetStatusRegister(void) {
	spiBufferTx[0] = NRF24L01P_SPI_CMD_NOP;
	writeToSPI(1);
	return spiBufferRx[0];
}
