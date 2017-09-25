#ifndef SYS_TIMER_H
#define SYS_TIMER_H		1

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "hal_common_includes.h"
#include "core_cm4.h"

#define sys_timerGetCurrentCycleMicros() (1000-(SysTick->VAL/72))

typedef struct {
	uint32_t sec;
	uint32_t us;
} sys_time_t;

void sys_timerInit(void);
void sys_timerUpdate(void);

uint64_t sys_timerMicros(void);
float sys_timerSeconds(void);
sys_time_t sys_timerGetTimeSinceEpoch(void);

#endif