/*
 * mission_timekeeper.c
 *
 *  Created on: Jan 14, 2015
 *      Author: aghosh01
 */

#include "mission_timekeeper.h"

static volatile time_val mission_time_counter;
static volatile uint16_t	delay_millis_to_go;
static volatile uint8_t 	delay_flag;

static volatile flag_data_struct flag_queue;

void init_mission_timekeeper(void)
{
	flag_queue.flag_id = 0U;
	flag_queue.flag_interval_millis = 0xFFFFU;
	flag_queue.millis_to_go = 0xFFFFU;
	flag_queue.next = NULL;
	flag_queue.state = STATE_NOT_SETUP;
	mission_time_counter.ms = 0U;
	mission_time_counter.seconds = 0;
	delay_millis_to_go = 0U;
	delay_flag = 0U;
}

void flag_scheduler_callback(void)
{
	flag_data_struct *headptr = &flag_queue;
	while(headptr != NULL)
	{
		if(headptr->state != STATE_NOT_SETUP)
		{
			--(headptr->millis_to_go);
			if(headptr->millis_to_go == 0U)
			{
				headptr->state = STATE_PENDING;
				headptr->millis_to_go = headptr->flag_interval_millis;
			}
			headptr = headptr->next;
		}
		else
		{
			headptr = headptr->next;
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

void timekeeper_delay(uint16_t millis)
{
	DISABLE_INTERRUPTS;
	delay_millis_to_go = millis;
	delay_flag = 1U;
	ENABLE_INTERRUPTS;
	while(delay_flag == 1U);
}

uint8_t create_flag(uint16_t interval_ms)
{
	DISABLE_INTERRUPTS;
	flag_data_struct *headptr = &flag_queue;
	while(headptr != NULL)
	{
		if(headptr->state == STATE_NOT_SETUP)
		{
			if(headptr->flag_id <= MAX_NUM_FLAGS)
			{
				headptr->state = STATE_PROCESSED;
				headptr->flag_interval_millis = interval_ms;
				headptr->millis_to_go = interval_ms;
				ENABLE_INTERRUPTS;
				return headptr->flag_id;
			}
		}
		else
		{
			if(headptr->flag_id >= MAX_NUM_FLAGS)
			{
				ENABLE_INTERRUPTS;
				return ERR_COULD_NOT_CREATE_FLAG;
			}
			if(headptr->next == NULL)
			{
				headptr->next = (flag_data_struct *)malloc(sizeof(flag_data_struct));
				headptr->next->flag_id = (headptr->flag_id)+1U;
				headptr->next->flag_interval_millis = interval_ms;
				headptr->next->millis_to_go = interval_ms;
				headptr->next->state = STATE_PROCESSED;
				headptr->next->next = NULL;
				ENABLE_INTERRUPTS;
				return headptr->next->flag_id;
			}
			else
			{
				headptr = headptr->next;
			}
		}
	}
	ENABLE_INTERRUPTS;
	return ERR_COULD_NOT_CREATE_FLAG;
}

flag_state get_flag_state(uint8_t flag_id)
{
	flag_data_struct *headptr = &flag_queue;
	while(headptr != NULL)
	{
		if(headptr->flag_id == flag_id)
		{
			return headptr->state;
		}
		headptr = headptr->next;
	}
	return ERR_FLAG_NOT_FOUND;
}

flag_state reset_flag(uint8_t flag_id)
{
	flag_data_struct *headptr = &flag_queue;
	while(headptr != NULL)
	{
		if(headptr->flag_id == flag_id)
		{
			headptr->state = STATE_PROCESSED;
			return FLAG_SET_SUCCESS;
		}
		headptr = headptr->next;
	}
	return ERR_FLAG_NOT_FOUND;
}
