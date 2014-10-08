/*
 * utils.h
 *
 *  Created on: Jun 4, 2014
 *      Author: raraujo
 */

#ifndef UTILS_H_
#define UTILS_H_
#include <stdint.h>

extern void timerDelayUs(uint32_t timeUs);
extern void timerDelayMs(uint32_t timeMs);
extern void resetDevice(void);

extern void enterReprogrammingMode(void);
#endif /* UTILS_H_ */
