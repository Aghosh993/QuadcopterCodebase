#ifndef CONSOLE_IO_H_
#define CONSOLE_IO_H_	1

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "system_shell.h"
#include "debug_ipc.h"

#define consoleProc_PRIO	2UL
#define consoleProc_pdMS	10

void consoleIOTask(void *pvParameters);
void dbgIOTask(void *pvParameters);

#endif