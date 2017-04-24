#ifndef INTERRUPT_UTILS_H
#define INTERRUPT_UTILS_H	1

#include "hal_common_includes.h"

#define _disable_interrupts() IntMasterDisable()
#define _enable_interrupts() IntMasterEnable()

#endif