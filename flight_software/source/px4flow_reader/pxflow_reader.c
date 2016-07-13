/*
 * pxflow_reader.c
 *
 *  Created on: Jan 22, 2015
 *      Author: aghosh01
 */

#include "px4flow_reader.h"

static pxflow_flow_data_struct last_received_flow_data;
static float last_received_height;

// Something received - print out all bytes and parse packet
mavlink_message_t msg;
mavlink_status_t status;
mavlink_optical_flow_t optical_flow;

float get_pxflow_height(void)
{
	return last_received_height;
}
void get_pxflow_flow_data(pxflow_flow_data_struct *buffer)
{
	buffer->x_velocity = last_received_flow_data.x_velocity;
	buffer->y_velocity = last_received_flow_data.y_velocity;
}

void px4flow_interrupt_callback(uint8 rxbyte)
{
	if (mavlink_parse_char(MAVLINK_COMM_0, rxbyte, &msg, &status))
	{
		if(msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW)
		{
			mavlink_msg_optical_flow_decode(&msg, &optical_flow);
			last_received_flow_data.x_velocity = optical_flow.flow_comp_m_x;
			last_received_flow_data.y_velocity = optical_flow.flow_comp_m_y;

			last_received_height = optical_flow.ground_distance;
		}
	}
}
