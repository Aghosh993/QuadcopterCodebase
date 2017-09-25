#include "sys_timer.h"

static volatile sys_time_t currentEpoch;

void sys_timerInit(void)
{
	currentEpoch.sec = 0U;
	currentEpoch.us = 0U;
}

void sys_timerUpdate(void)
{
	currentEpoch.us += 1000U;
	currentEpoch.sec += currentEpoch.us/1000000U;
	currentEpoch.us %= 1000000U;
}

uint64_t sys_timerMicros(void)
{
	return (uint64_t)(currentEpoch.sec * 1000000U) +
			(uint64_t)(currentEpoch.us) +
			(uint64_t)sys_timerGetCurrentCycleMicros();
}

float sys_timerSeconds(void)
{
	return (float)(currentEpoch.sec) + (float)(currentEpoch.us)/(float)1000000 +
				(float)sys_timerGetCurrentCycleMicros()/(float)1000000;
}

sys_time_t sys_timerGetTimeSinceEpoch(void)
{
	return currentEpoch;
}