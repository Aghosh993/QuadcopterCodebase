/*
 * mission_timekeeper.h
 *
 *  Created on: Jan 14, 2015
 *      Author: aghosh01
 */

#ifndef MISSION_TIMEKEEPER_H_
#define MISSION_TIMEKEEPER_H_

// #include <stdlib.h>
#include <stdint.h>

#include "interrupt_utils.h"

#define MAX_NUM_FLAGS				30U

typedef enum {
	STATE_NOT_SETUP,
	STATE_PENDING,
	STATE_PROCESSED,
	ERR_FLAG_NOT_FOUND,
	FLAG_SET_SUCCESS
} flag_state;

typedef struct {
	uint32_t seconds;
	uint16_t ms;
} time_val;

typedef struct {
	flag_state state;
	uint16_t millis_to_go;
	uint16_t flag_interval_millis;
} flag_data_struct;

void init_mission_timekeeper(void);

/*
	Task scheduling functions:
 */
uint8_t create_flag(uint16_t interval_ms);
flag_state get_flag_state(uint8_t flag_id);
flag_state reset_flag(uint8_t flag_id);

/*
	Delay/timekeeping functions:
 */
void zero_mission_time(void);
void set_mission_time(uint32_t sec, uint32_t msec);
time_val get_mission_time(void);
void timekeeper_delay(uint16_t millis);
float get_mission_time_sec(void);
int32_t get_mission_time_msec(void); // WARNING!! This time rolls over at 2^31 milliseconds or about 24 days. Should be sufficient for most applications...

/*
	Interrupt callback functions to be installed in a 1 kHz RTI/other Timer ISR
 */
void flag_scheduler_callback(void);
void update_mission_time_counter(void);

#endif /* MISSION_TIMEKEEPER_H_ */
