/*
 * error.h
 *
 *  Created on: Jun 30, 2014
 *      Author: raraujo
 */

#ifndef ERROR_H_
#define ERROR_H_
#include "chip.h"

#define TIMEOUT_ERROR		_BIT(7)
#define INSTRUCTION_ERROR	_BIT(6)
#define OVERLOAD_ERROR		_BIT(5)
#define CHECKSUM_ERROR		_BIT(4)
#define RANGE_ERROR			_BIT(3)
#define OVERHEATING_ERROR	_BIT(2)
#define ANGLE_LIMIT_ERROR	_BIT(1)
#define VOLTAGE_ERROR		_BIT(0)


void printErrorTable(void);
void printError(uint8_t error);

#endif /* ERROR_H_ */
