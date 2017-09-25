#include "debug_ipc.h"

QueueHandle_t msgQueue;

int dbgIPCInit(void)
{
	msgQueue = xQueueCreate((UBaseType_t)DBG_IPCQUEUE_DEPTH, sizeof(dbgQueueItem_t));
	if(msgQueue == 0)
	{
		return -1;
	}
	return 0;
}
int dbgIPCPushInt(char *tag, uint32_t taglen, int n)
{
	if(taglen > MAX_TAGLEN)
	{
		return -1;
	}
	dbgQueueItem_t msg;
	union {
		int input;
		uint8_t output[4];
	} conv;
	uint32_t i = 0;
	for(i=0U; i<taglen; i++)
	{
		msg.tag[i] = tag[i];
	}
	msg.msgID = ID_INTMSG;
	conv.input = n;
	for(i=0U; i<4; i++)
	{
		msg.databuf[i] = conv.output[i];
	}
	if(xQueueSendToBack(msgQueue, &msg, 0) != pdPASS)
	{
		return -1;
	}
	return 0;
}

int dbgIPCPushFloat(char *tag, uint32_t taglen, float n)
{
	if(taglen > MAX_TAGLEN)
	{
		return -1;
	}
	dbgQueueItem_t msg;
	union {
		float input;
		uint8_t output[4];
	} conv;
	uint32_t i = 0;
	for(i=0U; i<taglen; i++)
	{
		msg.tag[i] = tag[i];
	}
	msg.msgID = ID_FLTMSG;
	conv.input = n;
	for(i=0U; i<4; i++)
	{
		msg.databuf[i] = conv.output[i];
	}
	if(xQueueSendToBack(msgQueue, &msg, 0) != pdPASS)
	{
		return -1;
	}
	return 0;
}

void dbgIPCPopFromQueue(void)
{
	union {
		uint8_t input[4];
		float output;
	} u8_to_flt;

	union {
		uint8_t input[4];
		int output;
	} u8_to_int;

	dbgQueueItem_t msg;

	if(uxQueueMessagesWaiting(msgQueue) > 0)
	{
		xQueueReceive(msgQueue, &msg, 0);
		switch(msg.msgID)
		{
			case ID_INTMSG:
				u8_to_int.input[0] = msg.databuf[0];
				u8_to_int.input[1] = msg.databuf[1];
				u8_to_int.input[2] = msg.databuf[2];
				u8_to_int.input[3] = msg.databuf[3];
				printf("DBG: Got tag %s, type INT with value %d\r\n", msg.tag, u8_to_int.output);
				break;
			case ID_FLTMSG:
				u8_to_flt.input[0] = msg.databuf[0];
				u8_to_flt.input[1] = msg.databuf[1];
				u8_to_flt.input[2] = msg.databuf[2];
				u8_to_flt.input[3] = msg.databuf[3];
				printf("DBG: Got tag %s, type FLOAT with value %f\r\n", msg.tag, u8_to_flt.output);
				break;
		}
	}
}