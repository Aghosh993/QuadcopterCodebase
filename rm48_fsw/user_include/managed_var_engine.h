#ifndef MANAGED_VAR_ENGINE_H_
#define MANAGED_VAR_ENGINE_H_	1

#include <stdint.h>

#define MAX_DESC_STRING_LEN		10U
#define MAX_SYMTAB_LEN			30U

typedef enum {
	TYPE_UINT8,
	TYPE_INT32,
	TYPE_FLOAT
} var_data_type;

typedef struct {
	uint8_t descriptor_string[MAX_DESC_STRING_LEN];
	uint8_t descriptor_string_len;

	var_data_type data_type;

	void* data_ptr;
	uint8_t data_len;
} managed_var;

void init_managed_var_symtab(void);

void create_managed_string(uint8_t* descriptor, uint8_t descriptor_len, uint8_t* data, uint8_t data_len);
void create_managed_int32(uint8_t* descriptor, uint8_t descriptor_len, int32_t* data, uint8_t data_len);
void create_managed_float(uint8_t* descriptor, uint8_t descriptor_len, float* data, uint8_t data_len);

int find_managed_var(uint8_t* descriptor_search_key, uint8_t descriptor_len, var_data_type type);

int set_managed_string(uint8_t* descriptor, uint8_t descriptor_len, uint8_t* data, uint8_t buffer_len);
int set_managed_int32(uint8_t* descriptor, uint8_t descriptor_len, int32_t* data, uint8_t buffer_len);
int set_managed_float(uint8_t* descriptor, uint8_t descriptor_len, float* data, uint8_t buffer_len);

int get_managed_string(uint8_t* descriptor, uint8_t descriptor_len, uint8_t* data, uint8_t buffer_len);
int get_managed_int32(uint8_t* descriptor, uint8_t descriptor_len, int32_t* data, uint8_t buffer_len);
int get_managed_float(uint8_t* descriptor, uint8_t descriptor_len, float* data, uint8_t buffer_len);

#endif