#ifndef INTERRUPT_UTILS_H
#define INTERRUPT_UTILS_H	1

#include "hal_common_includes.h"

inline void _enable_interrupts_generic()
{
	__enable_irq();
}

inline void _disable_interrupts_generic()
{
	__disable_irq();
}

#endif