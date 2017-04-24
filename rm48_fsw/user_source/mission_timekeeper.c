/*
 * mission_timekeeper.c
 *
 *  Created on: January 14, 2015
 *	Revised on: November 25, 2016
 *  (c) Abhimanyu Ghosh
 */

#include "mission_timekeeper.h"

static volatile flag_data_struct flag_queue[MAX_NUM_FLAGS];
static volatile time_val mission_time_counter;

static volatile uint16_t delay_millis_to_go;
static volatile uint8_t delay_flag;
static volatile uint8_t flag_queue_depth;

void init_mission_timekeeper(void)
{
	uint8_t i = 0U;
	for(i=0U; i<MAX_NUM_FLAGS; ++i)
	{
		flag_queue[i].flag_interval_millis = 0xFFFFU;
		flag_queue[i].millis_to_go = 0xFFFFU;
		flag_queue[i].state = STATE_NOT_SETUP;
	}
	mission_time_counter.ms = 0U;
	mission_time_counter.seconds = 0;
	delay_millis_to_go = 0U;
	delay_flag = 0U;
	flag_queue_depth = 0U;
}

void flag_scheduler_callback(void)
{
	uint8_t i = 0U;
	for(i=0U; i<flag_queue_depth; ++i)
	{
		if(flag_queue[i].state != STATE_NOT_SETUP && flag_queue[i].state != STATE_PENDING)
		{
			if(flag_queue[i].millis_to_go > 0U)
			{
				--flag_queue[i].millis_to_go;
			}
			else
			{
				flag_queue[i].state = STATE_PENDING;
				flag_queue[i].millis_to_go = flag_queue[i].flag_interval_millis;
			}
		}
	}
}

void update_mission_time_counter(void)
{
	++mission_time_counter.ms;
	if(mission_time_counter.ms == 1000U)
	{
		++mission_time_counter.seconds;
		mission_time_counter.ms = 0U;
	}
	if(delay_flag == 1U)
	{
		--delay_millis_to_go;
		if(delay_millis_to_go == 0U)
		{
			delay_flag = 0U;
		}
	}
}

time_val get_mission_time(void)
{
	return mission_time_counter;
}

void zero_mission_time(void)
{
	_disable_interrupts();
		mission_time_counter.seconds = 0U;
		mission_time_counter.ms = 0U;
	_enable_interrupts();
}

void set_mission_time(uint32_t sec, uint32_t msec)
{
	_disable_interrupts();
		
	_enable_interrupts();
}

void timekeeper_delay(uint16_t millis)
{
	_disable_interrupts();
		delay_millis_to_go = millis;
		delay_flag = 1U;
	_enable_interrupts();
	while(delay_flag == 1U);
}

uint8_t create_flag(uint16_t interval_ms)
{
	_disable_interrupts();
		if(flag_queue_depth < MAX_NUM_FLAGS)
		{
			flag_queue[flag_queue_depth].state = STATE_PROCESSED;
			flag_queue[flag_queue_depth].millis_to_go = interval_ms;
			flag_queue[flag_queue_depth].flag_interval_millis = interval_ms;
		}
		++flag_queue_depth;
	_enable_interrupts();
	return flag_queue_depth-1;
}

flag_state get_flag_state(uint8_t flag_id)
{
	if(flag_id < flag_queue_depth)
	{
		return flag_queue[flag_id].state;
	}
	return ERR_FLAG_NOT_FOUND;
}

flag_state reset_flag(uint8_t flag_id)
{
	if(flag_id < flag_queue_depth)
	{
		flag_queue[flag_id].state = STATE_PROCESSED;
		return FLAG_SET_SUCCESS;
	}
	return ERR_FLAG_NOT_FOUND;
}

float get_mission_time_sec(void)
{
	time_val mission_time;
	mission_time = get_mission_time();
	return (float)mission_time.seconds + (float)mission_time.ms*(float)0.001f;
}

int32_t get_mission_time_msec(void)
{
	time_val mission_time;
	mission_time = get_mission_time();
	return (int32_t)mission_time.seconds*1000 + (int32_t)mission_time.ms;
}