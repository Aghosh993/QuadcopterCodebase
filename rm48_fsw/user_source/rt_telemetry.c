/*
	File: rt_telemetry.c

	(c) Abhimanyu Ghosh, 2016
 */

#include "rt_telemetry.h"

// static void rt_telemetry_push_to_tx_stack(rt_telemetry_comm_channel *ch, rt_telemetry_msg *m)
// {
// }

static void rt_telemetry_send_blocking(rt_telemetry_comm_channel *ch, rt_telemetry_msg *m, uint8_t data_payload_location, uint8_t data_payload_len, uint8_t bytes_per_payload_element)
{
	union {
		rt_telemetry_msg m;
		uint8_t output[sizeof(rt_telemetry_msg)];
	} convert_to_tx_byte_stream;

	convert_to_tx_byte_stream.m = *m;

	uint8_t i = 0U;
	uint16_t chksum = 0U;

	for(i=1U; i<2+m->alignment_union.contents.message_buffer[0U]; ++i)
	{
		chksum += convert_to_tx_byte_stream.output[i];
	}

	for(i=DESCRIPTOR_STRING_MAX_LEN+2; i<DESCRIPTOR_STRING_MAX_LEN+2+data_payload_location - (DESCRIPTOR_STRING_MAX_LEN + 2); ++i)
	{
		chksum += convert_to_tx_byte_stream.output[i];
	}

	for(i=data_payload_location; i<data_payload_location+bytes_per_payload_element*data_payload_len; ++i)
	{
		chksum += convert_to_tx_byte_stream.output[i];
	}

	chksum += TELEM_PACKET_CHECKSUM_OFFSET;

	convert_to_tx_byte_stream.m.alignment_union.contents.checksum[0] = chksum & 0xFF;
	convert_to_tx_byte_stream.m.alignment_union.contents.checksum[1] = (chksum >> 8) & 0xFF;

	// // Send start byte + descriptor string:
	serialport_send_data_buffer_blocking(ch->comm_port_ptr, convert_to_tx_byte_stream.output, 2+m->alignment_union.contents.message_buffer[0U]); 

	// // Send msg id, and all bytes going up to but excluding data payload:
	serialport_send_data_buffer_blocking(ch->comm_port_ptr, &(convert_to_tx_byte_stream.output[DESCRIPTOR_STRING_MAX_LEN+2]), data_payload_location - (DESCRIPTOR_STRING_MAX_LEN + 2));
	
	// // Send data payload:
	serialport_send_data_buffer_blocking(ch->comm_port_ptr, &(convert_to_tx_byte_stream.output[data_payload_location]), bytes_per_payload_element*data_payload_len);

	// // Send Checksum bytes:
	serialport_send_data_buffer_blocking(ch->comm_port_ptr, &(convert_to_tx_byte_stream.output[MAX_MSG_PAYLOAD_SIZE+1U]), 2U);

	// Send start byte + descriptor string:
	// serialport_send_data_buffer(ch->comm_port_ptr, convert_to_tx_byte_stream.output, 2+m->message_buffer[0U]); 

	// Send msg id, and all bytes going up to but excluding data payload:
	// serialport_send_data_buffer(ch->comm_port_ptr, &(convert_to_tx_byte_stream.output[DESCRIPTOR_STRING_MAX_LEN+2]), data_payload_location - (DESCRIPTOR_STRING_MAX_LEN + 2));
	
	// Send data payload:
	// serialport_send_data_buffer(ch->comm_port_ptr, &(convert_to_tx_byte_stream.output[data_payload_location]), bytes_per_payload_element*data_payload_len);

	// Send Checksum bytes:
	// serialport_send_data_buffer(ch->comm_port_ptr, &convert_to_tx_byte_stream.output[MAX_MSG_PAYLOAD_SIZE+1U], 2U); 
}

// static void telem_msg_string_push_to_tx_queue(rt_telemetry_comm_channel *ch, telem_msg_string *msg)
// {
// 	rt_telemetry_msg rt_msg;

// 	union {
// 		telem_msg_string m;
// 		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
// 	} convert_to_byte_stream;

// 	convert_to_byte_stream.m = *msg;

// 	rt_msg.start_byte = 's';

// 	uint8_t i = 0U;
// 	// uint16_t checksum = 0U;

// 	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE; ++i)
// 	{
// 		rt_msg.message_buffer[i] = convert_to_byte_stream.output[i];
// 		// checksum += convert_to_byte_stream.output[i];
// 	}
// 	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

// 	// rt_msg.checksum = checksum;

// 	rt_telemetry_push_to_tx_stack(ch, &rt_msg);
// }

// static void telem_msg_n_floats_push_to_tx_stack(rt_telemetry_comm_channel *ch, telem_msg_n_floats *msg)
// {
// 	rt_telemetry_msg rt_msg;

// 	union {
// 		telem_msg_n_floats m;
// 		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
// 	} convert_to_byte_stream;

// 	convert_to_byte_stream.m = *msg;

// 	rt_msg.start_byte = 's';

// 	uint8_t i = 0U;
// 	// uint16_t checksum = 0U;

// 	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE; ++i)
// 	{
// 		rt_msg.message_buffer[i] = convert_to_byte_stream.output[i];
// 		// checksum += convert_to_byte_stream.output[i];
// 	}
// 	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

// 	// rt_msg.checksum = checksum;

// 	rt_telemetry_push_to_tx_stack(ch, &rt_msg);
// }

// static void telem_msg_n_ints_push_to_tx_stack(rt_telemetry_comm_channel *ch, telem_msg_n_ints *msg)
// {
// 	rt_telemetry_msg rt_msg;

// 	union {
// 		telem_msg_n_ints m;
// 		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
// 	} convert_to_byte_stream;

// 	convert_to_byte_stream.m = *msg;

// 	rt_msg.start_byte = 's';

// 	uint8_t i = 0U;
// 	// uint16_t checksum = 0U;

// 	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE; ++i)
// 	{
// 		rt_msg.message_buffer[i] = convert_to_byte_stream.output[i];
// 		// checksum += convert_to_byte_stream.output[i];
// 	}
// 	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

// 	// rt_msg.checksum = checksum;

// 	rt_telemetry_push_to_tx_stack(ch, &rt_msg);
// }

// static void telem_msg_m_n_float_matrix_push_to_tx_stack(rt_telemetry_comm_channel *ch, telem_msg_m_n_float_matrix *msg)
// {
// 	rt_telemetry_msg rt_msg;

// 	union {
// 		telem_msg_m_n_float_matrix m;
// 		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
// 	} convert_to_byte_stream;

// 	convert_to_byte_stream.m = *msg;

// 	rt_msg.start_byte = 's';

// 	uint8_t i = 0U;
// 	// uint16_t checksum = 0U;

// 	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE; ++i)
// 	{
// 		rt_msg.message_buffer[i] = convert_to_byte_stream.output[i];
// 		// checksum += convert_to_byte_stream.output[i];
// 	}
// 	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

// 	// rt_msg.checksum = checksum;

// 	rt_telemetry_push_to_tx_stack(ch, &rt_msg);
// }

// static void telem_msg_m_n_int_matrix_push_to_tx_stack(rt_telemetry_comm_channel *ch, telem_msg_m_n_int_matrix *msg)
// {
// 	rt_telemetry_msg rt_msg;

// 	union {
// 		telem_msg_m_n_int_matrix m;
// 		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
// 	} convert_to_byte_stream;

// 	convert_to_byte_stream.m = *msg;

// 	rt_msg.start_byte = 's';

// 	uint8_t i = 0U;
// 	// uint16_t checksum = 0U;

// 	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE; ++i)
// 	{
// 		rt_msg.message_buffer[i] = convert_to_byte_stream.output[i];
// 		// checksum += convert_to_byte_stream.output[i];
// 	}
// 	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

// 	// rt_msg.checksum = checksum;

// 	rt_telemetry_push_to_tx_stack(ch, &rt_msg);
// }

static void telem_msg_string_send_blocking(rt_telemetry_comm_channel *ch, telem_msg_string *msg)
{
	rt_telemetry_msg rt_msg;

	union {
		telem_msg_string m;
		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
	} convert_to_byte_stream;

	convert_to_byte_stream.m = *msg;

	rt_msg.alignment_union.contents.start_byte = 's';

	uint8_t i = 0U;
	// uint16_t checksum = 0U;

	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE; ++i)
	{
		rt_msg.alignment_union.contents.message_buffer[i] = convert_to_byte_stream.output[i];
		// checksum += convert_to_byte_stream.output[i];
	}
	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

	rt_msg.alignment_union.contents.checksum[0] = 0U;
	rt_msg.alignment_union.contents.checksum[1] = 0U;

	rt_telemetry_send_blocking(ch, &rt_msg, DESCRIPTOR_STRING_MAX_LEN+4, msg->string_len, 1U);
}

static void telem_msg_n_floats_send_blocking(rt_telemetry_comm_channel *ch, telem_msg_n_floats *msg)
{
	rt_telemetry_msg rt_msg;

	union {
		telem_msg_n_floats m;
		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
	} convert_to_byte_stream;
	
	convert_to_byte_stream.m = *msg;

	rt_msg.alignment_union.contents.start_byte = 's';

	uint8_t i = 0U;
	// uint16_t checksum = 0U;
	uint8_t padding_offset = 4-(3U+DESCRIPTOR_STRING_MAX_LEN)%4;

	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE-padding_offset; ++i)
	{
		rt_msg.alignment_union.contents.message_buffer[i] = convert_to_byte_stream.output[i+padding_offset];
		// checksum += convert_to_byte_stream.output[i+padding_offset];
	}
	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

	rt_msg.alignment_union.contents.checksum[0] = 0U;
	rt_msg.alignment_union.contents.checksum[1] = 0U;

	rt_telemetry_send_blocking(ch, &rt_msg, DESCRIPTOR_STRING_MAX_LEN+4, msg->init_data.message_header.n_floats_len, 4U);
}

static void telem_msg_n_ints_send_blocking(rt_telemetry_comm_channel *ch, telem_msg_n_ints *msg)
{
	rt_telemetry_msg rt_msg;

	union {
		telem_msg_n_ints m;
		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
	} convert_to_byte_stream;

	convert_to_byte_stream.m = *msg;

	rt_msg.alignment_union.contents.start_byte = 's';

	uint8_t i = 0U;
	// uint16_t checksum = 0U;
	uint8_t padding_offset = 4-(3U+DESCRIPTOR_STRING_MAX_LEN)%4;

	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE-padding_offset; ++i)
	{
		rt_msg.alignment_union.contents.message_buffer[i] = convert_to_byte_stream.output[i+padding_offset];
		// checksum += convert_to_byte_stream.output[i+padding_offset];
	}
	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

	rt_msg.alignment_union.contents.checksum[0] = 0U;
	rt_msg.alignment_union.contents.checksum[1] = 0U;

	rt_telemetry_send_blocking(ch, &rt_msg, DESCRIPTOR_STRING_MAX_LEN+4, msg->init_data.message_header.n_ints_len, 4U);
}

static void telem_msg_m_n_float_matrix_send_blocking(rt_telemetry_comm_channel *ch, telem_msg_m_n_float_matrix *msg)
{
	rt_telemetry_msg rt_msg;

	union {
		telem_msg_m_n_float_matrix m;
		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
	} convert_to_byte_stream;

	convert_to_byte_stream.m = *msg;

	rt_msg.alignment_union.contents.start_byte = 's';

	uint8_t i = 0U;
	// uint16_t checksum = 0U;
	uint8_t padding_offset = 4-(4U+DESCRIPTOR_STRING_MAX_LEN)%4;

	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE - padding_offset; ++i)
	{
		rt_msg.alignment_union.contents.message_buffer[i] = convert_to_byte_stream.output[i+padding_offset];
		// checksum += convert_to_byte_stream.output[i+padding_offset];
	}
	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

	rt_msg.alignment_union.contents.checksum[0] = 0U;
	rt_msg.alignment_union.contents.checksum[1] = 0U;

	rt_telemetry_send_blocking(ch, &rt_msg, DESCRIPTOR_STRING_MAX_LEN+5, msg->init_data.message_header.m * msg->init_data.message_header.n, 4U);
}

static void telem_msg_m_n_int_matrix_send_blocking(rt_telemetry_comm_channel *ch, telem_msg_m_n_int_matrix *msg)
{
	rt_telemetry_msg rt_msg;

	union {
		telem_msg_m_n_int_matrix m;
		uint8_t output[MAX_MSG_PAYLOAD_SIZE];
	} convert_to_byte_stream;

	convert_to_byte_stream.m = *msg;

	rt_msg.alignment_union.contents.start_byte = 's';

	uint8_t i = 0U;
	// uint16_t checksum = 0U;
	uint8_t padding_offset = 4-(4U+DESCRIPTOR_STRING_MAX_LEN)%4;

	for(i=0U; i<MAX_MSG_PAYLOAD_SIZE - padding_offset; ++i)
	{
		rt_msg.alignment_union.contents.message_buffer[i] = convert_to_byte_stream.output[i+padding_offset];
		// checksum += convert_to_byte_stream.output[i+padding_offset];
	}
	// checksum += TELEM_PACKET_CHECKSUM_OFFSET;

	rt_msg.alignment_union.contents.checksum[0] = 0U;
	rt_msg.alignment_union.contents.checksum[1] = 0U;

	rt_telemetry_send_blocking(ch, &rt_msg, DESCRIPTOR_STRING_MAX_LEN+5, msg->init_data.message_header.m * msg->init_data.message_header.n, 4U);
}

void rt_telemetry_init_channel(rt_telemetry_comm_channel *ch, serialport *comm_port)
{
	ch->comm_port_ptr = comm_port;

	uint32_t i = 0U;
	uint32_t j = 0U;
	for(i=0U; i<MAX_TELEM_MSG_TX_QUEUE_DEPTH; ++i)
	{
		ch->tx_message_buffer[i].alignment_union.contents.start_byte = 0U;
		for(j=0U; j<MAX_MSG_PAYLOAD_SIZE; ++j)
		{
			ch->tx_message_buffer[i].alignment_union.contents.message_buffer[j] = 0U;
		}
		ch->tx_message_buffer[i].alignment_union.contents.checksum[0] = 0U;
		ch->tx_message_buffer[i].alignment_union.contents.checksum[1] = 0U;
	}

	ch->tx_buffer_latest_write_position = 0U;
	ch->tx_buffer_latest_read_position = 0U;

	ch->rx_buffer_latest_write_position = 0U;
	ch->rx_buffer_latest_read_position = 0U;

	ch->rx_state = STATE_GETTING_START;
	ch->rx_iterator = 0U;

	init_managed_var_symtab();
}

/*
	Not implemented yet:
 */

// void send_telem_msg_string(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, uint8_t *string_to_transmit, uint8_t string_len)
// {
// 	telem_msg_string s;
// 	create_telem_msg_string(&s, descriptor_string, descriptor_string_len, string_to_transmit, string_len);
// 	telem_msg_string_push_to_tx_queue(ch, &s);
// }

// void send_telem_msg_n_floats(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, float *data_payload, uint8_t n_floats_len)
// {
// 	telem_msg_n_floats s;
// 	create_telem_msg_n_floats(&s, descriptor_string, descriptor_string_len, data_payload, n_floats_len);
// 	telem_msg_n_floats_push_to_tx_stack(ch, &s);
// }

// void send_telem_msg_n_ints(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, int32_t *data_payload, uint8_t n_ints_len)
// {
// 	telem_msg_n_ints s;
// 	create_telem_msg_n_ints(&s, descriptor_string, descriptor_string_len, data_payload, n_ints_len);
// 	telem_msg_n_ints_push_to_tx_stack(ch, &s);
// }

// void send_telem_msg_m_n_float_matrix(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, float *data_payload, uint8_t m, uint8_t n)
// {
// 	telem_msg_m_n_float_matrix s;
// 	create_telem_msg_m_n_float_matrix(&s, descriptor_string, descriptor_string_len, data_payload, m, n);
// 	telem_msg_m_n_float_matrix_push_to_tx_stack(ch, &s);
// }

// void send_telem_msg_m_n_int_matrix(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, int32_t *data_payload, uint8_t m, uint8_t n)
// {
// 	telem_msg_m_n_int_matrix s;
// 	create_telem_msg_m_n_int_matrix(&s, descriptor_string, descriptor_string_len, data_payload, m, n);
// 	telem_msg_m_n_int_matrix_push_to_tx_stack(ch, &s);
// }

/*
	Working
 */
void send_telem_msg_string_blocking(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, uint8_t *string_to_transmit, uint8_t string_len)
{
	telem_msg_string s;
	create_telem_msg_string(&s, descriptor_string, descriptor_string_len, string_to_transmit, string_len);
	telem_msg_string_send_blocking(ch, &s);
}

void send_telem_msg_n_floats_blocking(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, float *data_payload, uint8_t n_floats_len)
{
	telem_msg_n_floats s;
	create_telem_msg_n_floats(&s, descriptor_string, descriptor_string_len, data_payload, n_floats_len);
	telem_msg_n_floats_send_blocking(ch, &s);
}

void send_telem_msg_n_ints_blocking(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, int32_t *data_payload, uint8_t n_ints_len)
{
	telem_msg_n_ints s;
	create_telem_msg_n_ints(&s, descriptor_string, descriptor_string_len, data_payload, n_ints_len);
	telem_msg_n_ints_send_blocking(ch, &s);
}

void send_telem_msg_m_n_float_matrix_blocking(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, float *data_payload, uint8_t m, uint8_t n)
{
	telem_msg_m_n_float_matrix s;
	create_telem_msg_m_n_float_matrix(&s, descriptor_string, descriptor_string_len, data_payload, m, n);
	telem_msg_m_n_float_matrix_send_blocking(ch, &s);
}

void send_telem_msg_m_n_int_matrix_blocking(rt_telemetry_comm_channel *ch, uint8_t *descriptor_string, uint8_t descriptor_string_len, int32_t *data_payload, uint8_t m, uint8_t n)
{
	telem_msg_m_n_int_matrix s;
	create_telem_msg_m_n_int_matrix(&s, descriptor_string, descriptor_string_len, data_payload, m, n);
	telem_msg_m_n_int_matrix_send_blocking(ch, &s);
}

// void process_telem_rx_buffer(rt_telemetry_comm_channel *ch)
// {
// 	uint8_t recv_buffer;
// 	while(serialport_receive_data_buffer(ch->comm_port_ptr, &recv_buffer, 1U) > 0)
// 	{
// 		switch(ch->rx_state)
// 		{
// 			case STATE_GETTING_START:
// 				if(recv_buffer == 's')
// 				{
// 					ch->rx_state = STATE_GETTING_DESC_STRING_LEN;
// 				}				
// 				break;
// 			case STATE_GETTING_DESC_STRING_LEN:
// 				ch->rx_message_buffer[0U].message_buffer[ch->rx_iterator] = recv_buffer;
// 				++ch->rx_iterator;
// 				ch->rx_state = STATE_GETTING_DESCRIPTOR_STRING;
// 				break;
// 			case STATE_GETTING_DESCRIPTOR_STRING:
// 				ch->rx_message_buffer[0U].message_buffer[ch->rx_iterator] = recv_buffer;
// 				++ch->rx_iterator;
// 				if(ch->rx_iterator > ch->rx_message_buffer[0U].message_buffer[0U]+1)
// 				{
// 					ch->rx_state = STATE_GETTING_MSG_ID;
// 				}
// 				break;
// 			case STATE_GETTING_MSG_ID:
// 				ch->rx_state = STATE_GETTING_MSG_LEN;
// 				break;
// 			case STATE_GETTING_MSG_LEN:
// 				ch->rx_state = STATE_GETTING_MESSAGE_PAYLOAD;
// 				break;
// 			case STATE_GETTING_MESSAGE_PAYLOAD:
// 				ch->rx_state = STATE_GETTING_CHECKSUM;
// 				break;
// 		}
// 	}
// }
