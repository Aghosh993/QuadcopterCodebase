/*
 * sf10_reader.h
 *
 *  Created on: Jul 2, 2015
 *      Author: aghosh01
 */

#ifndef SF10_READER_H_
#define SF10_READER_H_	1

#include <stdint.h>
#include <math.h>

// #include "sf10_hal_ap2v4_tm4c.h"
#include "mission_timekeeper.h"
#include "serial_comms_highlevel.h"

#define MESSAGE_BUFFER_MAX_LEN 	7
#define MAX_HEIGHT_SF10_A		25.00f
#define MAX_HEIGHT_SF11_C		120.00f

#define SF10_DATA_REQUEST_BYTE	'd'

typedef enum
{
	STATE_IDLE,
	STATE_SENT_DATA_REQUEST_BYTE,
	STATE_GETTING_DATA,
	STATE_GOT_NEW_MEASUREMENT,
	STATE_ERROR
} sf10_data_acquisition_state;

typedef enum
{
	RESOURCE_ALLOC_TO_ISR,
	RESOURCE_ALLOC_TO_USER_PROG,
	RESOURCE_FREE
} resource_allocator_enum;

typedef struct
{
	float last_received_height;
  float timestamp;
	uint8_t ncycles_timeout_limit;
	sf10_data_acquisition_state acq_st;

	uint8_t received_data_raw[MESSAGE_BUFFER_MAX_LEN];
	uint8_t message_buffer_iterator;

	float max_height_possible;

	resource_allocator_enum height_variable_lock;
	// void (*uart_tx_function_ptr)(uint8_t);
	serialport *sensor_port;

	int _received_new_data;
} sf10_sensor_data_handler;


void init_new_sf10_data_handler(sf10_sensor_data_handler *dst, uint8_t timeout_limit, float max_height_possible, serialport *port, serialport_desc hw_desc);
void request_sf10_sensor_update(sf10_sensor_data_handler *dh);
float get_last_sf10_sensor_height(sf10_sensor_data_handler *dh);
float get_last_sf10_timestamp(sf10_sensor_data_handler *dh);
int sf10_received_new_data(sf10_sensor_data_handler *dh); // returns 1 if new data received since last call to this function, returns 0 otherwise
float sf10_reader_check_measurement(float raw_data);
void sf10_reader_process_bytes(sf10_sensor_data_handler *dh);

#endif /* SF10_READER_H_ */
