/*
 * nixie_clock.h
 *
 *  Created on: 30.07.2017
 *      Author: kkap6
 */

#ifndef NIXIE_CLOCK_H_
#define NIXIE_CLOCK_H_

#include <stdint.h>
#include <stdbool.h>

/** Structure holding time. */
typedef struct  __attribute__ ((__packed__)) {
	/// year, in ex 2017
	uint16_t year;
	/// month 1..12
	uint8_t month;
	/// day of month 1..31
	uint8_t day;
	/// hours: 0..23
	uint8_t hours;
	/// minutes: 0..59
	uint8_t minutes;
	/// seconds: 0..59
	uint8_t seconds;
} ClkTime;

/**
 * Initialize and start clock.
 */
bool nixieClockInit(void);

/**
 * Set time and date of clock.
 */
void setClockTime(ClkTime receiveTime);

#endif /* NIXIE_CLOCK_H_ */
