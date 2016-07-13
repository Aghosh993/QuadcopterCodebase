/*
 * level0_comms.c
 *
 *  Created on: Dec 18, 2014
 *      Author: aghosh01
 */
#include <stdlib.h>

#include "level0_comms_hal_stm32.h"
#include "level0_comms.h"

binary_communications_manager *create_new_comms_manager(void)
{
	binary_communications_manager *manager = (binary_communications_manager *)malloc(sizeof(binary_communications_manager));
	return manager;
}

void init_comm_lib(binary_communications_manager *cm, void* transmit_function_uart)
{
		// Initialize transmit queue:
	cm->tx_queue_headptr = (data_packet_list *)malloc(sizeof(data_packet_list));
	init_transmit_queue(cm->tx_queue_headptr);

	cm->tx_queue_headptr->queue_lock = QUEUE_FREE;

	// Enable comms lib:
	cm->exit_comms_lib = COMMS_LIB_RUN;

	// Initialize the state machine and buffer counter:
	cm->msg_parse_state = STATE_GETTING_START_BYTE;
	cm->message_iterator = 0U;

	cm->byte_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;
	cm->float_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;
	cm->int_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;
	cm->joystick_struct_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;

	// Initialize the joystick struct:
	cm->last_joystick_update_received.btn = 0;
	cm->last_joystick_update_received.btn_pressed = 0;
	cm->last_joystick_update_received.x = 0.0f;
	cm->last_joystick_update_received.y = 0.0f;
	cm->last_joystick_update_received.yaw = 0.0f;
	cm->last_joystick_update_received.throttle = -1.0f;


	// Allocate memory for incoming data packet and swap space:
	cm->incoming_message_buffer = (data_packet *)malloc(sizeof(data_packet));
	cm->incoming_message_buffer->message_contents = (uint8_t *)malloc(MAX_MESSAGE_LEN*sizeof(uint8_t));

	cm->uart_transmit_byte = transmit_function_uart;
}

void initiate_comms(binary_communications_manager *cm)
{
	uint8_t contents_ack = 0x01;
	data_packet conversation_starter;
	conversation_starter.message_id = MSG_ID_ACK;
	conversation_starter.message_len = 1U;
	conversation_starter.message_contents = &contents_ack;
	conversation_starter.checksum = 0x76;

	send_data_packet(cm, &conversation_starter);
}

void deInit_comm_lib(binary_communications_manager *cm)
{
	cm->exit_comms_lib = COMMS_LIB_EXIT;
}

void uart_rx_callback(binary_communications_manager *cm, uint8_t byte_received)
{
	data_packet outgoing;
	switch(cm->msg_parse_state)
	{
	case STATE_GETTING_START_BYTE:
		if(byte_received == START_BYTE)
		{
			cm->msg_parse_state = STATE_GETTING_MESSAGE_ID;
		}
		break;
	case STATE_GETTING_MESSAGE_ID:
		cm->incoming_message_buffer->message_id = byte_received;
		cm->msg_parse_state = STATE_GETTING_MESSAGE_LEN;
		break;
	case STATE_GETTING_MESSAGE_LEN:
		cm->incoming_message_buffer->message_len = byte_received;
		cm->msg_parse_state = STATE_GETTING_MESSAGE_CONTENTS;
		break;
	case STATE_GETTING_MESSAGE_CONTENTS:
		if(cm->message_iterator < cm->incoming_message_buffer->message_len)
		{
			cm->incoming_message_buffer->message_contents[cm->message_iterator] = byte_received;
			++cm->message_iterator;
		}
		if(cm->message_iterator >= cm->incoming_message_buffer->message_len)
		{
			cm->message_iterator = 0U;
			cm->msg_parse_state = STATE_GETTING_CHECKSUM;
		}
		break;
	case STATE_GETTING_CHECKSUM:
		cm->incoming_message_buffer->checksum = byte_received;
		cm->msg_parse_state = STATE_GOT_FULL_MESSAGE;

		// We want to fall through to the next case for
		// message processing!

	case STATE_GOT_FULL_MESSAGE:
		if(verify_checksum(*(cm->incoming_message_buffer)) == CHECKSUM_PASS)
		{
			cm->msg_parse_state = STATE_NEW_MESSAGE_AVAILABLE;
			switch(cm->incoming_message_buffer->message_id)
			{
				case MSG_ID_ACK:
					cm->msg_parse_state = STATE_GETTING_START_BYTE;
					// while(outgoing_queue_lock != QUEUE_FREE);
					cm->tx_queue_headptr->queue_lock = QUEUE_ALLOCATE_RX_CALLBACK;
					outgoing = pop_from_transmit_queue(cm->tx_queue_headptr);
					cm->tx_queue_headptr->queue_lock = QUEUE_FREE;
					if(outgoing.message_id == MSG_ID_NO_MESSAGE) // TX Queue is empty
					{
						// initiate_comms(cm);
					}
					else
					{
						send_data_packet(cm, &outgoing);
						if(outgoing.message_contents != NULL)
						{
							free(outgoing.message_contents);
						}
					}
					break;
				case MSG_ID_FCN_CALL: 	// Not implemented yet
					cm->msg_parse_state = STATE_GETTING_START_BYTE;
					break;
				case MSG_ID_GET_VAR:	// Not implemented yet
					cm->msg_parse_state = STATE_GETTING_START_BYTE;
					break;
				case MSG_ID_SET_VAR:	// Not implemented yet
					cm->msg_parse_state = STATE_GETTING_START_BYTE;
					break;
				case MSG_ID_DATA_BYTE:
					cm->msg_parse_state = STATE_GETTING_START_BYTE;
					parse_byte(cm);
					outgoing = pop_from_transmit_queue(cm->tx_queue_headptr);
					if(outgoing.message_id == MSG_ID_NO_MESSAGE) // TX Queue is empty
					{
						// initiate_comms(cm);
					}
					else
					{
						send_data_packet(cm, &outgoing);
						if(outgoing.message_contents != NULL)
						{
							free(outgoing.message_contents);
						}
					}
					break;
				case MSG_ID_DATA_INT32:
					//parse_int32(cm);
					cm->msg_parse_state = STATE_GETTING_START_BYTE;
					break;
				case MSG_ID_DATA_FLOAT:
					parse_float(cm);
					cm->msg_parse_state = STATE_GETTING_START_BYTE;
					outgoing = pop_from_transmit_queue(cm->tx_queue_headptr);
					if(outgoing.message_id == MSG_ID_NO_MESSAGE) // TX Queue is empty
					{
						// initiate_comms(cm);
					}
					else
					{
						send_data_packet(cm, &outgoing);
						if(outgoing.message_contents != NULL)
						{
							free(outgoing.message_contents);
						}
					}
					break;
				case MSG_ID_JOYSTICK_UPDATE:
					parse_joystick_update(cm);
					cm->msg_parse_state = STATE_GETTING_START_BYTE;
					outgoing = pop_from_transmit_queue(cm->tx_queue_headptr);
					if(outgoing.message_id == MSG_ID_NO_MESSAGE) // TX Queue is empty
					{
						// initiate_comms(cm);
					}
					else
					{
						send_data_packet(cm, &outgoing);
						if(outgoing.message_contents != NULL)
						{
							free(outgoing.message_contents);
						}
					}
				case MSG_ID_POSITIONING_UPDATE:
					parse_positioning_update(cm);
					cm->msg_parse_state = STATE_GETTING_START_BYTE;
					outgoing = pop_from_transmit_queue(cm->tx_queue_headptr);
					if(outgoing.message_id == MSG_ID_NO_MESSAGE) // TX Queue is empty
					{
						// initiate_comms(cm);
					}
					else
					{
						send_data_packet(cm, &outgoing);
						if(outgoing.message_contents != NULL)
						{
							free(outgoing.message_contents);
						}
					}
					break;
			}
		}
		else
		{
			// printf("a\n");
			cm->msg_parse_state = STATE_ERR_CHECKSUM;
		}
		break;
	default:
		cm->msg_parse_state = STATE_GETTING_START_BYTE;
		break;
	}
}

void create_packet(data_packet *packet, uint8_t message_id, uint8_t message_len,
							uint8_t *message_contents)
{
	uint8_t i = 0U;
	unsigned short running_sum = 0U;

	packet->message_id = message_id;
	packet->message_len = message_len;
	packet->message_contents = message_contents;

	for(i=0U; i<packet->message_len; ++i)
	{
		running_sum += packet->message_contents[i];
	}
	running_sum += 0x75;
	packet->checksum = running_sum & 0xFF;
}

static void send_data_packet(binary_communications_manager *cm, data_packet *packet)
{
	cm->uart_transmit_byte(START_BYTE);
	cm->uart_transmit_byte(packet->message_id);
	cm->uart_transmit_byte(packet->message_len);

	uint8_t i = 0U;
	for(i=0U; i<packet->message_len; ++i)
	{
		cm->uart_transmit_byte(packet->message_contents[i]);
	}
	cm->uart_transmit_byte(packet->checksum);
}

// Send functions for various data types:

/*
 * Safely swap the addresses of the primary reception and post-processing packet data buffers:
 */

int prioritized_send_byte(binary_communications_manager *cm, uint8_t data, uint8_t priority)
{
	data_packet *packet_to_send = (data_packet *)malloc(sizeof(data_packet));
	uint8_t *output_buffer = (uint8_t *)malloc(sizeof(uint8_t));
	output_buffer[0] = data;
	create_packet(packet_to_send, MSG_ID_DATA_BYTE, LEN_BYTE, output_buffer);
	data_packet_list *new_headptr = prioritized_push_to_transmit_queue(packet_to_send, priority, cm->tx_queue_headptr);
	if(new_headptr == NULL)
	{
		free(output_buffer);
		free(packet_to_send);
		return -1;
	}
	else
	{
		cm->tx_queue_headptr = new_headptr;
		return 1;
	}
}

void send_byte(binary_communications_manager *cm, uint8_t data)
{
	data_packet *packet_to_send = (data_packet *)malloc(sizeof(data_packet));
	create_packet(packet_to_send, MSG_ID_DATA_BYTE, LEN_BYTE, &data);
	send_data_packet(cm, packet_to_send);
}

int prioritized_send_float(binary_communications_manager *cm, float data, uint8_t priority)
{
	union float_to_bytes
	{
		float input;
		uint8_t output[4];
	} convert;

	convert.input = data;

	data_packet *packet_to_send = (data_packet *)malloc(sizeof(data_packet));
	uint8_t *output_buffer = (uint8_t *)malloc(LEN_FLOAT*sizeof(uint8_t));

	uint8_t i = 0U;
	for(i=0U;i<LEN_FLOAT;++i)
	{
		output_buffer[i] = convert.output[i];
	}

	create_packet(packet_to_send, MSG_ID_DATA_FLOAT, LEN_FLOAT, output_buffer);

	data_packet_list *new_headptr = prioritized_push_to_transmit_queue(packet_to_send, priority, cm->tx_queue_headptr);

	if(new_headptr == NULL)
	{
		free(output_buffer);
		free(packet_to_send);
		return -1;
	}
	else
	{
		cm->tx_queue_headptr = new_headptr;
		return 1;
	}
}

void send_float(binary_communications_manager *cm, float data)
{
	union float_to_bytes
	{
		float input;
		uint8_t output[LEN_FLOAT];
	} convert;

	convert.input = data;

	data_packet packet_to_send;
	create_packet(&packet_to_send, MSG_ID_DATA_FLOAT, LEN_FLOAT, convert.output);
	// while(cm->msg_parse_state != STATE_GETTING_START_BYTE); // Does this work?? We'll find out :)
	send_data_packet(cm, &packet_to_send);
}

void send_float_triple(binary_communications_manager *cm, float x, float y, float z)
{
	union float_to_bytes
	{
		float input[3];
		uint8_t output[LEN_FLOAT_TRIPLE];
	} convert;

	convert.input[0] = x;
	convert.input[1] = y;
	convert.input[2] = z;

	data_packet packet_to_send;
	create_packet(&packet_to_send, MSG_ID_DATA_FLOAT_TRIPLE, LEN_FLOAT_TRIPLE, convert.output);
	send_data_packet(cm, &packet_to_send);
}


void send_signed_int(binary_communications_manager *cm, int data)
{
	union integer_to_bytes
	{
		int input;
		uint8_t output[4];
	} convert;

	convert.input = data;

	data_packet packet_to_send;
	create_packet(&packet_to_send, MSG_ID_DATA_INT32, LEN_INT32, convert.output);
	send_data_packet(cm, &packet_to_send);
}

int prioritized_send_joystick_update(binary_communications_manager *cm, uint8_t button_number, uint8_t button_pressed_bool, float x, float y, float yaw, float throttle, uint8_t priority)
{
	union float_to_bytes
	{
		float input;
		uint8_t output[4];
	} convert;

	uint8_t i = 0U;

	uint8_t *data_to_send = (uint8_t *)malloc(LEN_JOYSTICK_UPDATE_PACKET*sizeof(uint8_t));

	data_to_send[0] = button_number;
	data_to_send[1] = button_pressed_bool;

	convert.input = x;
	for(i=0U; i<4U; ++i)
	{
		data_to_send[i+2] = convert.output[i];
	}

	convert.input = y;
	for(i=0U; i<4U; ++i)
	{
		data_to_send[i+6] = convert.output[i];
	}

	convert.input = yaw;
	for(i=0U; i<4U; ++i)
	{
		data_to_send[i+10] = convert.output[i];
	}

	convert.input = throttle;
	for(i=0U; i<4U; ++i)
	{
		data_to_send[i+14] = convert.output[i];
	}

	data_packet *packet_to_send = (data_packet *)malloc(sizeof(data_packet));
	create_packet(packet_to_send, MSG_ID_JOYSTICK_UPDATE, LEN_JOYSTICK_UPDATE_PACKET, data_to_send);

	data_packet_list *new_headptr = prioritized_push_to_transmit_queue(packet_to_send, priority, cm->tx_queue_headptr);

	if(new_headptr == NULL)
	{
		free(data_to_send);
		free(packet_to_send);
		return -1;
	}
	else
	{
		cm->tx_queue_headptr = new_headptr;
		return 1;
	}
}

void send_joystick_update(binary_communications_manager *cm, uint8_t button_number, uint8_t button_pressed_bool, float x, float y, float yaw, float throttle)
{
	union float_to_bytes
	{
		float input;
		uint8_t output[4];
	} convert;

	uint8_t i = 0U;

	uint8_t data_to_send[18U];
	data_to_send[0] = button_number;
	data_to_send[1] = button_pressed_bool;

	convert.input = x;
	for(i=0U; i<4U; ++i)
	{
		data_to_send[i+2] = convert.output[i];
	}

	convert.input = y;
	for(i=0U; i<4U; ++i)
	{
		data_to_send[i+6] = convert.output[i];
	}

	convert.input = yaw;
	for(i=0U; i<4U; ++i)
	{
		data_to_send[i+10] = convert.output[i];
	}

	convert.input = throttle;
	for(i=0U; i<4U; ++i)
	{
		data_to_send[i+14] = convert.output[i];
	}

	data_packet *packet_to_send = (data_packet *)malloc(sizeof(data_packet));
	create_packet(packet_to_send, MSG_ID_JOYSTICK_UPDATE, LEN_JOYSTICK_UPDATE_PACKET, data_to_send);
	send_data_packet(cm, packet_to_send);
	free(packet_to_send); // Should fix potential memory leak..
}

void send_positioning_update(binary_communications_manager *cm, float vel_x, float vel_y, float optical_flow_x,
								float optical_flow_y, float height_sonar, float height_lidar, uint64_t timestamp)
{
	union float_to_bytes
	{
		float input;
		uint8_t output[4U];
	} convert;

	union long_to_bytes
	{
		uint64_t input;
		uint8_t output[8U];
	} convert_long;

	uint8_t i = 0U;
	uint8_t data_to_send[LEN_POSITIONING_UPDATE_PACKET];

	convert.input = vel_x;
	for(i = 0U; i < 4U; ++i)
	{
		data_to_send[i] = convert.output[i];
	}

	convert.input = vel_y;
	for(i = 0U; i < 4U; ++i)
	{
		data_to_send[i+4U] = convert.output[i];
	}

	convert.input = optical_flow_x;
	for(i = 0U; i < 4U; ++i)
	{
		data_to_send[i+8U] = convert.output[i];
	}

	convert.input = optical_flow_y;
	for(i = 0U; i < 4U; ++i)
	{
		data_to_send[i+12U] = convert.output[i];
	}

	convert.input = height_sonar;
	for(i = 0U; i < 4U; ++i)
	{
		data_to_send[i+16U] = convert.output[i];
	}

	convert.input = height_lidar;
	for(i = 0U; i < 4U; ++i)
	{
		data_to_send[i+20U] = convert.output[i];
	}

	convert_long.input = timestamp;
	for(i = 0U; i < 8U; ++i)
	{
		data_to_send[i+24U] = convert_long.output[i];
	}

	data_packet *packet_to_send = (data_packet *)malloc(sizeof(data_packet));
	create_packet(packet_to_send, MSG_ID_POSITIONING_UPDATE, LEN_POSITIONING_UPDATE_PACKET, data_to_send);
	send_data_packet(cm, packet_to_send);
	free(packet_to_send); // Should fix potential memory leak..
}

// Receive functions:

static void parse_byte(binary_communications_manager *cm)
{
	if(cm->incoming_message_buffer->message_id != MSG_ID_DATA_BYTE)
	{
		return;
	}
	if(cm->incoming_message_buffer->message_len != LEN_BYTE)
	{
		return;
	}
	else
	{
		//while(byte_access_flag == ACCESS_FLAG_ACCESS_BLOCKED && exit_comms_lib == COMMS_LIB_RUN);
		cm->last_byte_received = *(cm->incoming_message_buffer->message_contents);
		cm->byte_access_flag = ACCESS_FLAG_ACCESS_BLOCKED;
	}
}


uint8_t recv_byte(binary_communications_manager *cm)
{
	uint8_t retval = 0;
	while(cm->byte_access_flag == ACCESS_FLAG_ACCESS_ALLOWED && cm->exit_comms_lib == COMMS_LIB_RUN);
	retval = cm->last_byte_received;
	cm->byte_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;
	return retval;
}


static void parse_float(binary_communications_manager *cm)
{
	if(cm->incoming_message_buffer->message_id != MSG_ID_DATA_FLOAT)
	{
		return;
	}
	if(cm->incoming_message_buffer->message_len != LEN_FLOAT)
	{
		return;
	}
	else
	{
		union bytes_to_float
		{
			float output;
			uint8_t input[4];
		} convert;
		uint8_t i = 0U;
		for(i=0U; i<LEN_FLOAT; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i];
		}
		while(cm->float_access_flag == ACCESS_FLAG_ACCESS_BLOCKED && cm->exit_comms_lib == COMMS_LIB_RUN);
		cm->last_float_received = convert.output;
		cm->float_access_flag = ACCESS_FLAG_ACCESS_BLOCKED;
	}
}

float recv_float(binary_communications_manager *cm)
{
	float retval = 0.0f;
	while(cm->float_access_flag == ACCESS_FLAG_ACCESS_ALLOWED && cm->exit_comms_lib == COMMS_LIB_RUN);
	retval = cm->last_float_received;
	cm->float_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;
	return retval;
}

void parse_joystick_update(binary_communications_manager *cm)
{
	if(cm->incoming_message_buffer->message_id != MSG_ID_JOYSTICK_UPDATE)
	{
		return;
	}
	if(cm->incoming_message_buffer->message_len != LEN_JOYSTICK_UPDATE_PACKET)
	{
		return;
	}
	else
	{
		union bytes_to_float {
			float output;
			uint8_t input[4];
		} convert;

		uint8_t i = 0U;

//		while(joystick_struct_access_flag == ACCESS_FLAG_ACCESS_BLOCKED && exit_comms_lib == COMMS_LIB_RUN);
		cm->joystick_struct_access_flag = ACCESS_FLAG_ACCESS_BLOCKED;
		cm->last_joystick_update_received.btn = cm->incoming_message_buffer->message_contents[0];
		cm->last_joystick_update_received.btn_pressed = cm->incoming_message_buffer->message_contents[1];

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i+2];
		}
		cm->last_joystick_update_received.x = convert.output;

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i+6];
		}
		cm->last_joystick_update_received.y = convert.output;

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i+10];
		}
		cm->last_joystick_update_received.yaw = convert.output;

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i+14];
		}
		cm->last_joystick_update_received.throttle = convert.output;
		cm->joystick_struct_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;
	}
}

void recv_joystick_update(binary_communications_manager *cm, joystick_data_struct* ret_str)
{
//	joystick_data_struct* ret_str = (joystick_data_struct *)malloc(sizeof(joystick_data_struct));
//	while(joystick_struct_access_flag == ACCESS_FLAG_ACCESS_BLOCKED && exit_comms_lib == COMMS_LIB_RUN);

	ret_str->btn = cm->last_joystick_update_received.btn;
	ret_str->btn_pressed = cm->last_joystick_update_received.btn_pressed;
	ret_str->x = cm->last_joystick_update_received.x;
	ret_str->y = cm->last_joystick_update_received.y;
	ret_str->yaw = cm->last_joystick_update_received.yaw;
	ret_str->throttle = cm->last_joystick_update_received.throttle;

	cm->joystick_struct_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;
}

void parse_positioning_update(binary_communications_manager *cm)
{
	if(cm->incoming_message_buffer->message_id != MSG_ID_POSITIONING_UPDATE)
	{
		return;
	}
	if(cm->incoming_message_buffer->message_len != LEN_POSITIONING_UPDATE_PACKET)
	{
		return;
	}
	else
	{
		union bytes_to_float {
			float output;
			uint8_t input[4];
		} convert;

		union bytes_to_long {
			uint64_t output;
			uint8_t input[8U];
		} convert_long;

		uint8_t i = 0U;

		cm->positioning_struct_access_flag = ACCESS_FLAG_ACCESS_BLOCKED;

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i];
		}
		cm->last_positioning_update_received.x_vel = convert.output;

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i+4U];
		}
		cm->last_positioning_update_received.y_vel = convert.output;

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i+8U];
		}
		cm->last_positioning_update_received.x_optical_flow = convert.output;

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i+12U];
		}
		cm->last_positioning_update_received.y_optical_flow = convert.output;

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i+16U];
		}
		cm->last_positioning_update_received.sonar_height_measurement = convert.output;

		for(i=0U; i<4U; ++i)
		{
			convert.input[i] = cm->incoming_message_buffer->message_contents[i+20U];
		}
		cm->last_positioning_update_received.lidar_height_measurement = convert.output;

		for(i=0U; i<8U; ++i)
		{
			convert_long.input[i] = cm->incoming_message_buffer->message_contents[i+24U];
		}
		cm->last_positioning_update_received.flow_data_timestamp = convert_long.output;

		cm->positioning_struct_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;
	}
}

void recv_positioning_update(binary_communications_manager *cm, positioning_update_struct *ret_str)
{
	// while(cm->positioning_struct_access_flag == ACCESS_FLAG_ACCESS_ALLOWED && cm->exit_comms_lib == COMMS_LIB_RUN);
	ret_str->x_vel = cm->last_positioning_update_received.x_vel;
	ret_str->y_vel = cm->last_positioning_update_received.y_vel;
	ret_str->x_optical_flow = cm->last_positioning_update_received.x_optical_flow;
	ret_str->y_optical_flow = cm->last_positioning_update_received.y_optical_flow;
	ret_str->sonar_height_measurement = cm->last_positioning_update_received.sonar_height_measurement;
	ret_str->lidar_height_measurement = cm->last_positioning_update_received.lidar_height_measurement;
	ret_str->flow_data_timestamp = cm->last_positioning_update_received.flow_data_timestamp;
	cm->positioning_struct_access_flag = ACCESS_FLAG_ACCESS_ALLOWED;

}

checksum_verify_result verify_checksum(data_packet input)		// Pass by value since we're not modifying
																// the argument data structure, but rather
																// merely inspecting it.
{
	uint8_t i = 0U;
	uint16_t running_sum = 0U;

	for(i=0; i<input.message_len; ++i)
	{
		running_sum += input.message_contents[i];
	}
	running_sum += 0x75;
	return ((running_sum & 0xFF) == input.checksum) ? CHECKSUM_PASS:CHECKSUM_FAIL;
}

/*
	Priority queue code:
 */
void init_transmit_queue(data_packet_list *ll)
{
	ll->current_queue_depth = 0U;

	ll->node_data = NULL;
	ll->previous = NULL;
	ll->next = NULL;
}

int push_to_transmit_queue(data_packet *packet, uint8_t priority, data_packet_list *ll)
{
	if(ll->current_queue_depth < MAX_QUEUE_DEPTH)
	{
		data_packet_list *ll_iterator = ll;
		while(ll_iterator)
		{
			if(ll_iterator->node_data == NULL) // If this node is empty (i.e. list empty), store data to it
			{
				ll_iterator->node_data = packet;
				ll_iterator->node_tx_priority = priority;
				break;
			}
			else
			{
				if(ll_iterator->next != NULL)
				{
					ll_iterator = ll_iterator->next;
				}
				else
				{
					data_packet_list *new_list_element = (data_packet_list *)malloc(sizeof(data_packet_list));

					ll_iterator->next = new_list_element;

					new_list_element->previous = ll_iterator;
					new_list_element->next = NULL;

					new_list_element->node_data = packet;
					new_list_element->node_tx_priority = priority;
					break;
				}
			}
		}
		ll->current_queue_depth += 1U;
	return 1;
	}
	else
	{
		return -1;
	}
}

data_packet_list *prioritized_push_to_transmit_queue(data_packet *packet, uint8_t priority, data_packet_list *ll)
{
//	while(outgoing_queue_lock != QUEUE_FREE);
	ll->queue_lock = QUEUE_ALLOCATE_PUSH_OP;

	if(ll->current_queue_depth < MAX_QUEUE_DEPTH)
	{
		data_packet_list *ll_iterator = ll;
		while(ll_iterator)
		{
			if(ll_iterator->node_data == NULL) // If this node is empty (i.e. list empty), store data to it
			{
				ll_iterator->node_data = packet;
				ll_iterator->node_tx_priority = priority;

				ll->current_queue_depth += 1U;
				ll->queue_lock = QUEUE_FREE;
				return ll;
			}
			if(priority < ll_iterator->node_tx_priority)
			{
				data_packet_list *new_list_element = (data_packet_list *)malloc(sizeof(data_packet_list));
				new_list_element->node_data = packet;
				new_list_element->node_tx_priority = priority;

				new_list_element->previous = NULL;
				new_list_element->next = ll_iterator;
				ll_iterator->previous = new_list_element;

				ll->current_queue_depth += 1U;
				ll->queue_lock = QUEUE_FREE;

				return new_list_element;
			}
			else
			{
				if(ll_iterator->next == NULL)
				{
					data_packet_list *new_list_element = (data_packet_list *)malloc(sizeof(data_packet_list));

					if(priority >= ll_iterator->node_tx_priority)
					{
						ll_iterator->next = new_list_element;

						new_list_element->previous = ll_iterator;
						new_list_element->next = NULL;

						new_list_element->node_data = packet;
						new_list_element->node_tx_priority = priority;
					}
					else
					{
						ll_iterator->previous = new_list_element;

						new_list_element->previous = NULL;
						new_list_element->next = ll_iterator;

						new_list_element->node_data = packet;
						new_list_element->node_tx_priority = priority;
					}
					ll->current_queue_depth += 1U;
					ll->queue_lock = QUEUE_FREE;
					return ll;
				}
				else
				{
					if(priority >= ll_iterator->node_tx_priority && priority <= ll_iterator->next->node_tx_priority)
					{
						data_packet_list *new_list_element = (data_packet_list *)malloc(sizeof(data_packet_list));

						ll_iterator->next->previous = new_list_element;

						new_list_element->previous = ll_iterator;
						new_list_element->next = ll_iterator->next;

						ll_iterator->next = new_list_element;

						new_list_element->node_data = packet;
						new_list_element->node_tx_priority = priority;

						ll->current_queue_depth += 1U;
						ll->queue_lock = QUEUE_FREE;

						return ll;
					}
					else
					{
						ll_iterator = ll_iterator->next;
					}
				}
			}
		}
	}
	else
	{
		ll->queue_lock = QUEUE_FREE;

		return NULL;
	}
	ll->queue_lock = QUEUE_FREE;
	return NULL;
}

data_packet_list *get_last_ll_element(data_packet_list *ll)
{
	data_packet_list *ll_iterator = ll;
	while(ll_iterator->next)
	{
		ll_iterator = ll_iterator->next;
	}
	return ll_iterator;
}

data_packet_list *get_first_ll_element(data_packet_list *ll)
{
	data_packet_list *ll_iterator = ll;
	while(ll_iterator->previous)
	{
		ll_iterator = ll_iterator->previous;
	}
	return ll_iterator;
}

data_packet pop_from_transmit_queue(data_packet_list *ll)
{
	data_packet return_value;
	data_packet_list *ll_iterator = get_last_ll_element(ll);

	if(ll_iterator->node_data != NULL)
	{
		// Copy down node data:
		return_value.message_id = ll_iterator->node_data->message_id;
		return_value.message_len = ll_iterator->node_data->message_len;
		return_value.message_contents = (uint8_t *)malloc(return_value.message_len * sizeof(uint8_t));
		uint8_t i = 0U;
		for(i=0U; i<return_value.message_len; ++i)
		{
			return_value.message_contents[i] = ll_iterator->node_data->message_contents[i];
		}
		return_value.checksum = ll_iterator->node_data->checksum;

		// Free node data message contents:
		if(ll_iterator->node_data->message_contents != NULL)
		{
			free(ll_iterator->node_data->message_contents);
			ll_iterator->node_data->message_contents = NULL;
		}

		// Free node data:
		free(ll_iterator->node_data);

		ll_iterator->node_data = NULL;

		// Move iterator one step back, if possible:
		if(ll_iterator->previous != NULL)
		{
			ll_iterator = ll_iterator->previous;

			// Free the last node, now that we no longer need it:
			free(ll_iterator->next);

			// Set "next" node of "previous" (i.e. currently pointed-to node by iterator) to NULL
			ll_iterator->next = NULL;
		}
		ll->current_queue_depth -= 1U;
	}
	else // List is empty:
	{
		return_value.message_id = MSG_ID_NO_MESSAGE;
		return_value.message_len = 0U;
		return_value.message_contents = NULL;
		return_value.checksum = 0x75;
	}

	return return_value;
}

data_packet_list* find_highest_priority_message(data_packet_list *ll)
{
	data_packet_list* retval = ll;
	data_packet_list* iterator = ll;
	while(iterator)
	{
		if(iterator->node_tx_priority > retval->node_tx_priority)
		{
			retval = iterator;
		}
		iterator = iterator->next;
	}
	return retval;
}

data_packet_list* find_lowest_priority_message(data_packet_list *ll)
{
	data_packet_list* retval = ll;
	data_packet_list* iterator = ll;
	while(iterator)
	{
		if(iterator->node_tx_priority < retval->node_tx_priority)
		{
			retval = iterator;
		}
		iterator = iterator->next;
	}
	return retval;
}

#ifdef RUNNING_IN_LINUX_SIMULATION_ENV
void print_from_queue(binary_communications_manager *cm)
{
	printf("Start of queue dump\n");
	data_packet_list *ll_iterator = cm->tx_queue_headptr;
	while(ll_iterator)
	{
		if(ll_iterator->node_data == NULL)
		{
			printf("List is empty!!\n");
			break;
		}
		printf("Node has message of type: ");
		if(ll_iterator->node_data->message_id == MSG_ID_ACK)
		{
			printf("MSG_ID_ACK ");
		}
		if (ll_iterator->node_data->message_id == MSG_ID_DATA_BYTE)
		{
			printf("MSG_ID_DATA_BYTE ");
		}
		if (ll_iterator->node_data->message_id == MSG_ID_DATA_FLOAT)
		{
			printf("MSG_ID_DATA_FLOAT, ");
		}

		printf("Checksum %d, Data %d, and priority: %d\n", ll_iterator->node_data->checksum, ll_iterator->node_data->message_contents[0], ll_iterator->node_tx_priority);

		ll_iterator = ll_iterator->next;

	}
	printf("End of queue dump\n");
}
#endif
