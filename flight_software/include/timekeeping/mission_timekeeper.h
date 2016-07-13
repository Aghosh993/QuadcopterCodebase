/*
 * mission_timekeeper.h
 *
 *  Created on: Jan 14, 2015
 *      Author: aghosh01
 */

#ifndef MISSION_TIMEKEEPER_H_
#define MISSION_TIMEKEEPER_H_

#include <stdlib.h>
#include <stdint.h>

#include <interrupts.h>

#define DISABLE_INTERRUPTS _disable_interrupts()
#define ENABLE_INTERRUPTS _enable_interrupts()

#define MAX_NUM_FLAGS				254
#define ERR_COULD_NOT_CREATE_FLAG 	255

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

typedef struct flag_struct_fwd_dec {
	uint8_t flag_id;
	flag_state state;
	uint16_t millis_to_go;
	uint16_t flag_interval_millis;
	struct flag_struct_fwd_dec *next;
} flag_data_struct;

void init_mission_timekeeper(void);
void flag_scheduler_callback(void);
uint8_t create_flag(uint16_t interval_ms);
flag_state get_flag_state(uint8_t flag_id);
flag_state reset_flag(uint8_t flag_id);

void update_mission_time_counter(void);
time_val get_mission_time(void);
void timekeeper_delay(uint16_t millis);

#endif /* MISSION_TIMEKEEPER_H_ */
