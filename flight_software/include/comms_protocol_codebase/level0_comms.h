/*
 * level0_comms.h
 *
 *  Created on: Dec 18, 2014
 *      Author: aghosh01
 */

#ifndef LEVEL0_COMMS_H_
#define LEVEL0_COMMS_H_

#include "level0_comms_hal_stm32.h"

/*
	Autopilot includes (not needed on linux!!):
 */
// #include "stdlib.h"
// #include "sci.h"

#define START_BYTE 					(uint8_t)'s'
#define MAX_MESSAGE_LEN 			255 	// In bytes

#define LEN_BYTE					1U
#define LEN_FLOAT					4U
#define LEN_FLOAT_TRIPLE			12U
#define LEN_INT32					4U

#define LEN_JOYSTICK_UPDATE_PACKET		18U
#define LEN_POSITIONING_UPDATE_PACKET	32U

#define MSG_ID_ACK					0
#define MSG_ID_FCN_CALL 			1
#define MSG_ID_GET_VAR				2
#define MSG_ID_SET_VAR				3
#define MSG_ID_DATA_BYTE			4
#define MSG_ID_DATA_FLOAT			5
#define MSG_ID_DATA_INT32			6
#define MSG_ID_JOYSTICK_UPDATE		7
#define MSG_ID_POSITIONING_UPDATE	8
#define MSG_ID_DATA_FLOAT_TRIPLE	9
#define MSG_ID_NO_MESSAGE			99

#define DATA_RECV_MAX_WAIT_CYCLES 	100000

#define ACCESS_FLAG_ACCESS_ALLOWED 		1
#define ACCESS_FLAG_ACCESS_BLOCKED 		0

#define MAX_QUEUE_DEPTH					50

#define QUEUE_ALLOCATE_RX_CALLBACK 		0
#define QUEUE_ALLOCATE_PUSH_OP			1
#define QUEUE_FREE						2

#define COMMS_LIB_RUN 	0
#define COMMS_LIB_EXIT 	1

typedef struct {
	uint8_t 	message_id;
	uint8_t 	message_len;
	uint8_t 	*message_contents;
	uint8_t 	checksum;
} data_packet;

typedef struct {
	uint8_t btn;
	uint8_t btn_pressed;
	float x;
	float y;
	float yaw;
	float throttle;
} joystick_data_struct;

typedef struct {
	float x_vel;
	float y_vel;
	float x_optical_flow;
	float y_optical_flow;
	float sonar_height_measurement;
	float lidar_height_measurement;
	uint64_t flow_data_timestamp;
} positioning_update_struct;

typedef enum {
	STATE_GETTING_START_BYTE,
	STATE_GETTING_MESSAGE_ID,
	STATE_GETTING_MESSAGE_LEN,
	STATE_GETTING_MESSAGE_CONTENTS,
	STATE_GETTING_CHECKSUM,
	STATE_GOT_FULL_MESSAGE,
	STATE_NEW_MESSAGE_AVAILABLE,
	STATE_MESSAGE_RETRIEVED,
	STATE_ERR_CHECKSUM,
	STATE_ERR_UNKNOWN
} incoming_msg_parse_state;

typedef enum {
	CHECKSUM_PASS,
	CHECKSUM_FAIL
} checksum_verify_result;

typedef enum {
	STATUS_GOT_RAW_DATA,
	DATA_RECV_SUCCESS,
	ERR_DATA_TYPE_MISMATCH,
	ERR_DATA_LEN_MISMATCH,
	ERR_TIMEOUT
} data_recv_status;

typedef struct node
{
	data_packet *node_data;
	uint8_t node_tx_priority;
	struct node *next;
	struct node *previous;

	uint8_t queue_lock;
	uint8_t current_queue_depth;
} data_packet_list;

typedef void (*serial_send_func_ptr) (uint8_t);

typedef struct
{
	data_packet_list *tx_queue_headptr;
	/*
		Not used at the moment:
	 */
	data_packet_list *rx_queue_headptr;

	incoming_msg_parse_state msg_parse_state;
	uint8_t message_iterator;

	data_packet *incoming_message_buffer;

	uint8_t last_byte_received;
	uint8_t byte_access_flag;

	float last_float_received;
	uint8_t float_access_flag;

	int last_int_received;
	uint8_t int_access_flag;

	joystick_data_struct last_joystick_update_received;
	uint8_t joystick_struct_access_flag;

	positioning_update_struct last_positioning_update_received;
	uint8_t positioning_struct_access_flag;

	uint8_t exit_comms_lib;

	serial_send_func_ptr uart_transmit_byte;
} binary_communications_manager;

// Auxiliary/helper functions:

void uart_rx_callback(binary_communications_manager *cm, uint8_t byte_received);

void initiate_comms(binary_communications_manager *cm);

void deInit_comm_lib(binary_communications_manager *cm);
checksum_verify_result verify_checksum(data_packet input);

void create_packet(data_packet *packet, uint8_t message_id, uint8_t message_len,
							uint8_t *message_contents);
static void send_data_packet(binary_communications_manager *cm, data_packet *packet);

void send_byte(binary_communications_manager *cm, uint8_t data);
int prioritized_send_byte(binary_communications_manager *cm, uint8_t data, uint8_t priority);

void send_float(binary_communications_manager *cm, float data);
int prioritized_send_float(binary_communications_manager *cm, float data, uint8_t priority);

void send_signed_int(binary_communications_manager *cm, int data);

void send_joystick_update(binary_communications_manager *cm, uint8_t button_number, uint8_t button_pressed_bool, float x, float y, float yaw, float throttle);
int prioritized_send_joystick_update(binary_communications_manager *cm, uint8_t button_number, uint8_t button_pressed_bool, float x, float y, float yaw, float throttle, uint8_t priority);

void send_positioning_update(binary_communications_manager *cm, float vel_x, float vel_y, float optical_flow_x,
								float optical_flow_y, float height_sonar, float height_lidar, uint64_t timestamp);

uint8_t recv_byte(binary_communications_manager *cm);
static void parse_byte(binary_communications_manager *cm);

int recv_int32(binary_communications_manager *cm);
static void parse_int32(binary_communications_manager *cm);

float recv_float(binary_communications_manager *cm);
void send_float_triple(binary_communications_manager *cm, float x, float y, float z);
static void parse_float(binary_communications_manager *cm);

void recv_joystick_update(binary_communications_manager *cm, joystick_data_struct* ret_str);
void parse_joystick_update(binary_communications_manager *cm);

void recv_positioning_update(binary_communications_manager *cm, positioning_update_struct *ret_str);
void parse_positioning_update(binary_communications_manager *cm);

int recv_signed_int(binary_communications_manager *cm, data_recv_status *status);

void init_comm_lib(binary_communications_manager *cm, void* transmit_function_uart);

binary_communications_manager *create_new_comms_manager(void);
void init_transmit_queue(data_packet_list *ll);
int push_to_transmit_queue(data_packet *packet, uint8_t priority, data_packet_list *ll);
data_packet_list *prioritized_push_to_transmit_queue(data_packet *packet, uint8_t priority, data_packet_list *ll);
data_packet_list *get_last_ll_element(data_packet_list *ll);
data_packet_list *get_first_ll_element(data_packet_list *ll);
data_packet pop_from_transmit_queue(data_packet_list *ll);
data_packet_list* find_highest_priority_message(data_packet_list *ll);
data_packet_list* find_lowest_priority_message(data_packet_list *ll);
data_packet pop_from_transmit_queue_highest_priority(data_packet_list* ll);

#ifdef RUNNING_IN_LINUX_SIMULATION_ENV
	void print_from_queue(binary_communications_manager *cm);
#endif
#endif /* LEVEL0_COMMS_H_ */
