#include "managed_var_engine.h"

static managed_var var_array[MAX_SYMTAB_LEN];
static uint16_t var_table_depth;

static void create_managed_var_generic(managed_var* v, uint8_t* descriptor, uint8_t descriptor_len)
{
	uint8_t i = 0U;
	for(i=0U; i<descriptor_len; ++i)
	{
		v->descriptor_string[i] = descriptor[i];
	}
	
	v->descriptor_string_len = descriptor_len;
}

static int compare_var_descriptor(managed_var *v, uint8_t* search_string, uint8_t search_string_len)
{
	uint8_t i = 0U;

	if(!v)
	{
		return -1;
	}
	if(v->descriptor_string_len != search_string_len)
	{
		return -1;
	}
	else
	{
		for(i=0U; i<search_string_len; ++i)
		{
			if(v->descriptor_string[i] != search_string[i])
			{
				return -1;
			}
		}
		return 0;
	}
}

void init_managed_var_symtab(void)
{
	var_table_depth = 0U;
}

void create_managed_string(uint8_t* descriptor, uint8_t descriptor_len, uint8_t* data, uint8_t data_len)
{
	managed_var *dst = &(var_array[var_table_depth]);
	create_managed_var_generic(dst, descriptor, descriptor_len);

	dst->data_type = TYPE_UINT8;

	dst->data_ptr = (void *)data;
	dst->data_len = data_len;
	var_table_depth += 1U;
}

void create_managed_int32(uint8_t* descriptor, uint8_t descriptor_len, int32_t* data, uint8_t data_len)
{
	managed_var *dst = &(var_array[var_table_depth]);
	create_managed_var_generic(dst, descriptor, descriptor_len);

	dst->data_type = TYPE_INT32;

	dst->data_ptr = (void *)data;
	dst->data_len = data_len;
	var_table_depth += 1U;
}

void create_managed_float(uint8_t* descriptor, uint8_t descriptor_len, float* data, uint8_t data_len)
{
	managed_var *dst = &(var_array[var_table_depth]);
	create_managed_var_generic(dst, descriptor, descriptor_len);

	dst->data_type = TYPE_FLOAT;

	dst->data_ptr = (void *)data;
	dst->data_len = data_len;
	var_table_depth += 1U;
}

int find_managed_var(uint8_t* descriptor_search_key, uint8_t descriptor_len, var_data_type type)
{
	uint16_t i = 0U;
	for(i=0U; i<var_table_depth; ++i)
	{
		if(var_array[i].data_type == type)
		{
			if(compare_var_descriptor(&(var_array[i]), descriptor_search_key, descriptor_len) == 0)
			{
				return i;
			}
		}
	}
	return -1;
}

int set_managed_string(uint8_t* descriptor, uint8_t descriptor_len, uint8_t* data, uint8_t buffer_len)
{
	int dest_position = find_managed_var(descriptor, descriptor_len, TYPE_UINT8);
	
	if(dest_position < 0)
	{
		return -1;
	}
	
	uint16_t dest_position_u16 = (uint16_t)dest_position;

	managed_var *dst = &(var_array[dest_position_u16]);
	uint8_t *dst_data_buffer = (uint8_t *)(dst->data_ptr);

	if(dst->data_len < buffer_len)
	{
		return -1;
	}

	uint8_t i = 0U;

	for(i=0U; i<buffer_len; ++i)
	{
		dst_data_buffer[i] = data[i];
	}

	return 0;
}

int set_managed_int32(uint8_t* descriptor, uint8_t descriptor_len, int32_t* data, uint8_t buffer_len)
{
	int dest_position = find_managed_var(descriptor, descriptor_len, TYPE_INT32);
	
	if(dest_position < 0)
	{
		return -1;
	}
	
	uint16_t dest_position_u16 = (uint16_t)dest_position;

	managed_var *dst = &(var_array[dest_position_u16]);
	uint32_t *dst_data_buffer = (int32_t *)(dst->data_ptr);

	if(dst->data_len < buffer_len)
	{
		return -1;
	}

	uint8_t i = 0U;

	for(i=0U; i<buffer_len; ++i)
	{
		dst_data_buffer[i] = data[i];
	}
	
	return 0;
}

int set_managed_float(uint8_t* descriptor, uint8_t descriptor_len, float* data, uint8_t buffer_len)
{
	int dest_position = find_managed_var(descriptor, descriptor_len, TYPE_FLOAT);
	
	if(dest_position < 0)
	{
		return -1;
	}
	
	uint16_t dest_position_u16 = (uint16_t)dest_position;

	managed_var *dst = &(var_array[dest_position_u16]);
	float *dst_data_buffer = (float *)(dst->data_ptr);

	if(dst->data_len < buffer_len)
	{
		return -1;
	}

	uint8_t i = 0U;

	for(i=0U; i<buffer_len; ++i)
	{
		dst_data_buffer[i] = data[i];
	}
	
	return 0;
}

int get_managed_string(uint8_t* descriptor, uint8_t descriptor_len, uint8_t* data, uint8_t buffer_len)
{
	int src_position = find_managed_var(descriptor, descriptor_len, TYPE_UINT8);
	
	if(src_position < 0)
	{
		return -1;
	}
	
	uint16_t src_position_u16 = (uint16_t)src_position;

	managed_var *src = &(var_array[src_position_u16]);
	uint8_t *src_data_buffer = (uint8_t *)(src->data_ptr);

	if(src->data_len > buffer_len)
	{
		return -1;
	}

	uint8_t i = 0U;

	for(i=0U; i<src->data_len; ++i)
	{
		data[i] = src_data_buffer[i];
	}

	return 0;
}

int get_managed_int32(uint8_t* descriptor, uint8_t descriptor_len, int32_t* data, uint8_t buffer_len)
{
	int src_position = find_managed_var(descriptor, descriptor_len, TYPE_INT32);
	
	if(src_position < 0)
	{
		return -1;
	}
	
	uint16_t src_position_u16 = (uint16_t)src_position;

	managed_var *src = &(var_array[src_position_u16]);
	int32_t *src_data_buffer = (int32_t *)(src->data_ptr);

	if(src->data_len > buffer_len)
	{
		return -1;
	}

	uint8_t i = 0U;

	for(i=0U; i<src->data_len; ++i)
	{
		data[i] = src_data_buffer[i];
	}

	return 0;
}

int get_managed_float(uint8_t* descriptor, uint8_t descriptor_len, float* data, uint8_t buffer_len)
{
	int src_position = find_managed_var(descriptor, descriptor_len, TYPE_FLOAT);
	
	if(src_position < 0)
	{
		return -1;
	}
	
	uint16_t src_position_u16 = (uint16_t)src_position;

	managed_var *src = &(var_array[src_position_u16]);
	float *src_data_buffer = (float *)(src->data_ptr);

	if(src->data_len > buffer_len)
	{
		return -1;
	}

	uint8_t i = 0U;

	for(i=0U; i<src->data_len; ++i)
	{
		data[i] = src_data_buffer[i];
	}

	return 0;
}