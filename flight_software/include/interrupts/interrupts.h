#ifndef INTERRUPTS_H
#define INTERRUPTS_H 1

#include <hal_common_includes.h>

void _enable_interrupts(void);
void _disable_interrupts(void);

/*
	ISR Setup function prototypes:
 */
void systick_setup(void);

#endif