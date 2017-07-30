/*
 * nixie_clock.c
 *
 *  Created on: 30.07.2017
 *      Author: kkap6
 */

#include "nixie_clock.h"
#include "nrf_log.h"
#include "app_timer.h"

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */

#define CLOCK_INTERVAL	1000 // Interval of clock in ms

/// Array with number of days for each month (index is equal to month number)
static uint8_t daysInMonth[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31,
		30, 31 };

/// Date for starting session by button
static ClkTime actualTime = { 0 };

APP_TIMER_DEF(updateActualTimeId);

/**
 * This function update actual time every second.
 */
static void updateActualTime(void *p_context) {

	if (60 == ++actualTime.seconds) {
		actualTime.seconds = 0;
		if (60 == ++actualTime.minutes) {
			actualTime.minutes = 0;
			if (24 == ++actualTime.hours) {
				actualTime.hours = 0;
				if (2 != actualTime.month) {
					if (++actualTime.day
							== (daysInMonth[actualTime.month] + 1)) {
						actualTime.day = 1;
						if (13 == ++actualTime.month) {
							actualTime.month = 1;
							++actualTime.year;
						}
					}
				} else if ((0 == actualTime.year % 4
						&& 0 != actualTime.year % 100)
						|| 0 == actualTime.year % 400) {
					if (++actualTime.day
							== (daysInMonth[actualTime.month] + 2)) {
						actualTime.day = 1;
						++actualTime.month;
					}
				} else if (!((0 == actualTime.year % 4
						&& 0 != actualTime.year % 100)
						|| 0 == actualTime.year % 400)) {
					if (++actualTime.day
							== (daysInMonth[actualTime.month] + 1)) {
						actualTime.day = 1;
						++actualTime.month;
					}
				}
			}
		}
	}
	NRF_LOG_PRINTF("%d-%d-%d\t%d:%d:%d\r\n", actualTime.year, actualTime.month,
			actualTime.day, actualTime.hours, actualTime.minutes,
			actualTime.seconds);
}

bool nixieClockInit(void) {
	uint32_t err_code = app_timer_create(&updateActualTimeId,
			APP_TIMER_MODE_REPEATED, updateActualTime);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(updateActualTimeId,
			APP_TIMER_TICKS(CLOCK_INTERVAL, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
	return true;
}

void setClockTime(ClkTime receiveTime) {
	actualTime.year = receiveTime.year;
	actualTime.month = receiveTime.month;
	actualTime.day = receiveTime.day;
	actualTime.hours = receiveTime.hours;
	actualTime.minutes = receiveTime.minutes;
	actualTime.seconds = receiveTime.seconds;

}
