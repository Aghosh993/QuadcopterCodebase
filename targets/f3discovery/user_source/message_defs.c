/*
	File: message_defs.c

	(c) Abhimanyu Ghosh, 2016
 */

#include "message_defs.h"

void create_telem_msg_string(telem_msg_string *dst, uint8_t *descriptor_string, uint8_t descriptor_string_len, uint8_t *string_to_transmit, uint8_t string_len)
{
	dst->descriptor_string_len = descriptor_string_len;

	uint32_t i = 0U;
	for(i=0U; i<descriptor_string_len; ++i)
	{
		dst->descriptor_string[i] = descriptor_string[i];
	}

	for(i=descriptor_string_len; i<DESCRIPTOR_STRING_MAX_LEN; ++i)
	{
		dst->descriptor_string[i] = 0U;
	}

	dst->msg_id = TELEM_MSG_ID_STRING;

	dst->string_len = string_len;

	for(i=0U; i<string_len; ++i)
	{
		dst->string_to_transmit[i] = string_to_transmit[i];
	}
	for(i=string_len; i<MAX_MSG_PAYLOAD_SIZE - DESCRIPTOR_STRING_MAX_LEN - 3U; ++i)
	{
		dst->string_to_transmit[i] = 0U;
	}
}

void create_telem_msg_n_floats(telem_msg_n_floats *dst, uint8_t *descriptor_string, uint8_t descriptor_string_len, float *data_payload, uint8_t n_floats_len)
{
	dst->init_data.message_header.descriptor_string_len = descriptor_string_len;

	uint32_t i = 0U;
	for(i=0U; i<descriptor_string_len; ++i)
	{
		dst->init_data.message_header.descriptor_string[i] = descriptor_string[i];
	}

	dst->init_data.message_header.msg_id = TELEM_MSG_ID_N_FLOATS;

	dst->init_data.message_header.n_floats_len = n_floats_len;

	for(i=0U; i<n_floats_len; ++i)
	{
		dst->data_payload_convert.data_payload[i] = data_payload[i];
	}
}

void create_telem_msg_n_ints(telem_msg_n_ints *dst, uint8_t *descriptor_string, uint8_t descriptor_string_len, int32_t *data_payload, uint8_t n_ints_len)
{
	dst->init_data.message_header.descriptor_string_len = descriptor_string_len;

	uint32_t i = 0U;
	for(i=0U; i<descriptor_string_len; ++i)
	{
		dst->init_data.message_header.descriptor_string[i] = descriptor_string[i];
	}

	dst->init_data.message_header.msg_id = TELEM_MSG_ID_N_INTS;

	dst->init_data.message_header.n_ints_len = n_ints_len;

	for(i=0U; i<n_ints_len; ++i)
	{
		dst->data_payload_convert.data_payload[i] = data_payload[i];
	}
}

void create_telem_msg_m_n_float_matrix(telem_msg_m_n_float_matrix *dst, uint8_t *descriptor_string, uint8_t descriptor_string_len, float *data_payload, uint8_t m, uint8_t n)
{
	dst->init_data.message_header.descriptor_string_len = descriptor_string_len;

	uint32_t i = 0U;
	for(i=0U; i<descriptor_string_len; ++i)
	{
		dst->init_data.message_header.descriptor_string[i] = descriptor_string[i];
	}

	dst->init_data.message_header.msg_id = TELEM_MSG_ID_M_N_FLOAT_MATRIX;

	dst->init_data.message_header.m = m;
	dst->init_data.message_header.n = n;

	for(i=0U; i<m*n; ++i)
	{
		dst->data_payload_convert.data_payload[i] = data_payload[i];
	}
}

void create_telem_msg_m_n_int_matrix(telem_msg_m_n_int_matrix *dst, uint8_t *descriptor_string, uint8_t descriptor_string_len, int32_t *data_payload, uint8_t m, uint8_t n)
{
	dst->init_data.message_header.descriptor_string_len = descriptor_string_len;

	uint32_t i = 0U;
	for(i=0U; i<descriptor_string_len; ++i)
	{
		dst->init_data.message_header.descriptor_string[i] = descriptor_string[i];
	}

	dst->init_data.message_header.msg_id = TELEM_MSG_ID_M_N_INT_MATRIX;

	dst->init_data.message_header.m = m;
	dst->init_data.message_header.n = n;

	for(i=0U; i<m*n; ++i)
	{
		dst->data_payload_convert.data_payload[i] = data_payload[i];
	}
}