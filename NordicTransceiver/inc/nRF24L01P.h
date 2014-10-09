/**
 * @file nRF24L01P.h
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

#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * The state variable is a Bitfield
 * since it will make it easier for us to know which command to send depending on the mode
 * Bit 0  - Power mode
 * Bit 1 - RX mode
 * Bit 2 - TX mode
 * Bit 3 - NO ACK - only applicable on TX mode
 */

typedef enum {
	NRF24L01P_MODE_UNKNOWN = -1,
	NRF24L01P_MODE_POWER_DOWN = 0,
	NRF24L01P_MODE_STANDBY,
	NRF24L01P_MODE_RX = 0x2,
	NRF24L01P_MODE_TX = 0x4,
	NRF24L01P_MODE_TX_NO_ACK = 0x8, // no ACK in TX mode
} nRF24L01P_Mode_Type;

/*
 * The following FIFOs are present in nRF24L01+:
 *   TX three level, 32 byte FIFO
 *   RX three level, 32 byte FIFO
 */
#define NRF24L01P_TX_FIFO_COUNT   3
#define NRF24L01P_RX_FIFO_COUNT   3

#define NRF24L01P_TX_FIFO_SIZE   32
#define NRF24L01P_TX_ACK_FIFO_SIZE   15 // This only applies to 2 Mbps and an ARD of 250 uS
#define NRF24L01P_RX_FIFO_SIZE   32

#define NRF24L01P_SPI_MAX_DATA_RATE     10000000

#define NRF24L01P_SPI_CMD_RD_REG            0x00
#define NRF24L01P_SPI_CMD_WR_REG            0x20
#define NRF24L01P_SPI_CMD_RD_RX_PAYLOAD     0x61
#define NRF24L01P_SPI_CMD_WR_TX_PAYLOAD     0xa0
#define NRF24L01P_SPI_CMD_FLUSH_TX          0xe1
#define NRF24L01P_SPI_CMD_FLUSH_RX          0xe2
#define NRF24L01P_SPI_CMD_REUSE_TX_PL       0xe3
#define NRF24L01P_SPI_CMD_R_RX_PL_WID       0x60
#define NRF24L01P_SPI_CMD_W_ACK_PAYLOAD     0xa8
#define NRF24L01P_SPI_CMD_W_TX_PYLD_NO_ACK  0xb0
#define NRF24L01P_SPI_CMD_NOP               0xff

#define NRF24L01P_REG_CONFIG                0x00
#define NRF24L01P_REG_EN_AA                 0x01
#define NRF24L01P_REG_EN_RXADDR             0x02
#define NRF24L01P_REG_SETUP_AW              0x03
#define NRF24L01P_REG_SETUP_RETR            0x04
#define NRF24L01P_REG_RF_CH                 0x05
#define NRF24L01P_REG_RF_SETUP              0x06
#define NRF24L01P_REG_STATUS                0x07
#define NRF24L01P_REG_OBSERVE_TX            0x08
#define NRF24L01P_REG_RPD                   0x09
#define NRF24L01P_REG_RX_ADDR_P0            0x0a
#define NRF24L01P_REG_RX_ADDR_P1            0x0b
#define NRF24L01P_REG_RX_ADDR_P2            0x0c
#define NRF24L01P_REG_RX_ADDR_P3            0x0d
#define NRF24L01P_REG_RX_ADDR_P4            0x0e
#define NRF24L01P_REG_RX_ADDR_P5            0x0f
#define NRF24L01P_REG_TX_ADDR               0x10
#define NRF24L01P_REG_RX_PW_P0              0x11
#define NRF24L01P_REG_RX_PW_P1              0x12
#define NRF24L01P_REG_RX_PW_P2              0x13
#define NRF24L01P_REG_RX_PW_P3              0x14
#define NRF24L01P_REG_RX_PW_P4              0x15
#define NRF24L01P_REG_RX_PW_P5              0x16
#define NRF24L01P_REG_FIFO_STATUS           0x17
#define NRF24L01P_REG_DYNPD                 0x1c
#define NRF24L01P_REG_FEATURE               0x1d

// STATUS register:
#define NRF24L01P_STATUS_TX_FULL        (1<<0)
#define NRF24L01P_STATUS_RX_P_NO        (0x7<<1)
#define NRF24L01P_STATUS_MAX_RT         (1<<4)
#define NRF24L01P_STATUS_TX_DS          (1<<5)
#define NRF24L01P_STATUS_RX_DR          (1<<6)

#define NRF24L01P_TX_PWR_ZERO_DB         0
#define NRF24L01P_TX_PWR_MINUS_6_DB     -6
#define NRF24L01P_TX_PWR_MINUS_12_DB   -12
#define NRF24L01P_TX_PWR_MINUS_18_DB   -18

#define NRF24L01P_DATARATE_250_KBPS    250
#define NRF24L01P_DATARATE_1_MBPS     1000
#define NRF24L01P_DATARATE_2_MBPS     2000

#define NRF24L01P_CRC_NONE               0
#define NRF24L01P_CRC_8_BIT              8
#define NRF24L01P_CRC_16_BIT            16

#define NRF24L01P_CRC_NONE               0
#define NRF24L01P_CRC_8_BIT              8
#define NRF24L01P_CRC_16_BIT            16

#define NRF24L01P_MIN_RF_FREQUENCY    2400
#define NRF24L01P_MAX_RF_FREQUENCY    2525

#define NRF24L01P_PIPE_P0                0
#define NRF24L01P_PIPE_P1                1
#define NRF24L01P_PIPE_P2                2
#define NRF24L01P_PIPE_P3                3
#define NRF24L01P_PIPE_P4                4
#define NRF24L01P_PIPE_P5                5

#define DEFAULTNRF24L01P_ADDRESS       ((unsigned long long) 0xE7E7E7 )
#define DEFAULTNRF24L01P_ADDRESS_WIDTH  3
#define DEFAULTNRF24L01P_CRC            NRF24L01P_CRC_8_BIT
#define DEFAULTNRF24L01P_RF_FREQUENCY  (NRF24L01P_MIN_RF_FREQUENCY + 100)
#define DEFAULTNRF24L01P_DATARATE       NRF24L01P_DATARATE_2_MBPS
#define DEFAULTNRF24L01P_TX_PWR         NRF24L01P_TX_PWR_ZERO_DB
#define DEFAULTNRF24L01P_TRANSFER_SIZE  32

struct NRF24L01P {
	uint32_t mode;
	int payloadsinTXFIFO;
	volatile int transmissionTimeout;
};

extern struct NRF24L01P nRF24L01P;

/**
 * nRF24L01+ Single Chip 2.4GHz Transceiver from Nordic Semiconductor.
 *
 * @param transmitMode if 0 it will start in receive mode
 */
void nRF24L01PInit(unsigned char transmitMode);

/**
 * Function to be called on the main loop.
 *
 * Checks for timeouts in data in the buffer and checks for new IRQs
 */
void RFiterate();

/**
 * Set the RF frequency.
 *
 * @param frequency the frequency of RF transmission in MHz (2400..2525).
 */
void RFsetFrequency(int frequency);

/**
 * Get the RF frequency.
 *
 * @return the frequency of RF transmission in MHz (2400..2525).
 */
int RFgetFrequency(void);

/**
 * Set the RF output power.
 *
 * @param power the RF output power in dBm (0, -6, -12 or -18).
 */
void RFsetRfOutputPower(int power);

/**
 * Get the RF output power.
 *
 * @return the RF output power in dBm (0, -6, -12 or -18).
 */
int RFgetRfOutputPower(void);

/**
 * Set the Air data rate.
 *
 * @param rate the air data rate in kbps (250, 1M or 2M).
 */
void RFsetAirDataRate(int rate);

/**
 * Get the Air data rate.
 *
 * @return the air data rate in kbps (250, 1M or 2M).
 */
int RFgetAirDataRate(void);

/**
 * Set the CRC width.
 *
 * @param width the number of bits for the CRC (0, 8 or 16).
 */
void RFsetCrcWidth(int width);

/**
 * Get the CRC width.
 *
 * @return the number of bits for the CRC (0, 8 or 16).
 */
int RFgetCrcWidth(void);

/**
 * Set the Receive address.
 *
 * @param address address associated with the particular pipe DEFAULTNRF24L01P_ADDRESS
 * @param width width of the address in bytes (3..5) DEFAULTNRF24L01P_ADDRESS_WIDTH
 * @param pipe pipe to associate the address with (0..5, default 0) NRF24L01P_PIPE_P0
 *
 * Note that Pipes 0 & 1 have 3, 4 or 5 byte addresses,
 *  while Pipes 2..5 only use the lowest byte (bits 7..0) of the
 *  address provided here, and use 2, 3 or 4 bytes from Pipe 1's address.
 *  The width parameter is ignored for Pipes 2..5.
 */
void RFsetRxAddress(unsigned long long address, int width, int pipe);

/**
 * Set the Transmit address.
 *
 * @param address address for transmission DEFAULTNRF24L01P_ADDRESS
 * @param width width of the address in bytes (3..5) DEFAULTNRF24L01P_ADDRESS, DEFAULTNRF24L01P_ADDRESS_WIDTH
 *
 * Note that the address width is shared with the Receive pipes,
 *  so a change to that address width affect transmissions.
 */
void RFsetTxAddress(unsigned long long address, int width);

/**
 * Get the Receive address.
 *
 * @param pipe pipe to get the address from (0..5, default 0)NRF24L01P_PIPE_P0
 * @return the address associated with the particular pipe
 */
unsigned long long RFgetRxAddress(int pipe);

/**
 * Get the Transmit address.
 *
 * @return address address for transmission
 */
unsigned long long RFgetTxAddress(void);

/**
 * Set the transfer size.
 *
 * @param size the size of the transfer, in bytes (1..32) DEFAULTNRF24L01P_TRANSFER_SIZE
 * @param pipe pipe for the transfer (0..5, default 0) NRF24L01P_PIPE_P0
 */
void RFsetTransferSize(int size, int pipe);

/**
 * Get the transfer size.
 *
 * @return the size of the transfer, in bytes (1..32). NRF24L01P_PIPE_P0
 */
int RFgetTransferSize(int pipe);

/**
 * Put the nRF24L01+ into Receive mode
 */
void RFsetReceiveMode(void);

/**
 * Put the nRF24L01+ into Transmit mode
 */
void RFsetTransmitMode(void);

/**
 * Power up the nRF24L01+ into Standby mode
 */
void RFpowerUp(void);

/**
 * Power down the nRF24L01+ into Power Down mode
 */
void RFpowerDown(void);

/**
 * Enable the nRF24L01+ to Receive or Transmit (using the CE pin)
 */
void RFenable(void);

/**
 * Disable the nRF24L01+ to Receive or Transmit (using the CE pin)
 */
void RFdisable(void);

/**
 * In case of failed transmission, force the transceiver to retransmit
 * the last payload
 */
void RFretransmistLastPayload();

/**
 * Transmit data
 *
 * @param data pointer to an array of bytes to write
 * @param count the number of bytes to send (1..32)
 * @return the number of bytes actually written, or -1 for an error
 */
int RFpushDataToBuffer(unsigned char *data, int count);

/**
 * Checks if the library ring buffer is empty
 *
 * @return true if the library TX buffer is empty
 */
bool RFisTXBufferEmpty();

/**
 * Move data from the library buffer to the Nordic chip
 * Depending on the state of the TX FIFO, data may be moved or not
 *
 * @param pipe Only used in RX mode, it specifies which pipe
 * 			   is the ACK payload for.
 * @return true if data was actually moved
 */
bool RFmoveBufferToTransmission(int pipe);

/**
 * Determine if there is data available to read
 *
 * @param pipe the receive pipe to check for data
 * @return true if the is data waiting in the given pipe
 */
bool RFreadable(int pipe);

/**
 * Disable all receive pipes
 *
 * Note: receive pipes are enabled when their address is set.
 */
void RFdisableAllRxPipes(void);

/**
 * Disable AutoAcknowledge function
 */
void RFdisableAutoAcknowledge(void);

/**
 * Enable AutoAcknowledge function
 *
 * @param pipe the receive pipe NRF24L01P_PIPE_P0
 */
void RFenableAutoAcknowledge(int pipe);

/**
 * Enable the NO ACK transmission mode
 *
 * @param enable if true, packets will be sent without waiting for ack
 */
void RFenableNoACKTX(bool enable);

/**
 * Disable AutoRetransmit function
 */
void RFdisableAutoRetransmit(void);

/**
 * Enable AutoRetransmit function
 *
 * @param delay the delay between restransmits, in uS (250uS..4000uS)
 * @param count number of retransmits before generating an error (1..15)
 */
void RFenableAutoRetransmit(int delay, int count);

/**
 * Check if there is data on the RX FIFO
 *
 * @return true if the FIFO is empty
 */
bool RFisRXFIFOEmpty();

/**
 * Get the current.
 *
 * @return -1 if the value is invalid and the FIFO should be flushed,
 *  or the number of bytes in the payload
 */
int RFgetRXPayloadSize();

/**
 * Read a payload from the RX FIFO
 *
 * @param data pointer to the reception buffer
 * @param count count of bytes
 */
void RFRXFIFOread(char *data, int count);

/**
 * Flushes the RX FIFO
 */
void RFFlushRXFIFO();

/**
 * Flushes the TX FIFO
 */
void RFFlushTXFIFO();
/**
 * Get the contents of an addressable register.
 *
 * @param regAddress address of the register
 * @return the contents of the register
 */
int RFgetRegister(int regAddress);

/**
 * Set the contents of an addressable register.
 *
 * @param regAddress address of the register
 * @param regData data to write to the register
 */
void RFsetRegister(int regAddress, int regData);

/**
 * Get the contents of the status register.
 *
 * @return the contents of the status register
 */
int RFgetStatusRegister(void);

#endif /* __NRF24L01P_H__ */
