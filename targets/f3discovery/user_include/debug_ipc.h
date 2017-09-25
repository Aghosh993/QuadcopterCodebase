#ifndef DEBUG_IPC_H_
#define DEBUG_IPC_H_	1

/*
	Standard includes:
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/*
	FreeRTOS headers:
 */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define DBG_IPCQUEUE_DEPTH	50U
#define MAX_TAGLEN			15U

typedef enum {
	ID_INTMSG,
	ID_FLTMSG,
	ID_STRMSG
} msgID_t;

typedef struct {
	char tag[MAX_TAGLEN];
	msgID_t msgID;
	uint8_t databuf[4];
} dbgQueueItem_t;

int dbgIPCInit(void);
int dbgIPCPushInt(char *tag, uint32_t taglen, int n);
int dbgIPCPushFloat(char *tag, uint32_t taglen, float n);
void dbgIPCPopFromQueue(void);
// int dbgIPCPushString(char *tag, int taglen, char *str, int msglen);

#endif