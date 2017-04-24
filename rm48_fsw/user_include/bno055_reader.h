#ifndef BNO055_H
#define BNO055_H 1

#include <hal_common_includes.h>
#include <serial_comms_highlevel.h>

/* BNO055 UART protocol

		Register write:

			Byte 1		Start Byte		0xAA
			Byte 2		Write 			0x00
			Byte 3 		Reg address		-
			Byte 4		Length			-
			Byte 5...	Data 			-

		ACK response:

			Byte 1		Header			0xEE
			Byte 2		Status			0x01: WRITE_SUCCESS
										0x03: WRITE_FAIL
										0x04: REGMAP_INVALID_ADDRESS
										0x05: REGMAP_WRITE_DISABLED
										0x06: WRONG_START_BYTE
										0x07: BUS_OVER_RUN_ERROR
										0X08: MAX_LENGTH_ERROR
										0x09: MIN_LENGTH_ERROR
										0x0A: RECEIVE_CHARACTER_TIMEOUT

		Register read:

			Byte 1		Start Byte 		0xAA
			Byte 2 		Read 			0x01
			Byte 3 		Reg address 	-
			Byte 4		Length 			-

		Read success response:

			Byte 1 		Response Byte 	0xBB
			Byte 2 		Length 			-
			Byte 3 ...	Data 			-

		Read failure or ACK response:

			Byte 1 		Header 			0xEE
			Byte 2 		Status 			0x02: READ_FAIL
										0x04: REGMAP_INVALID_ADDRESS
										0x05: REGMAP_WRITE_DISABLED
										0x06: WRONG_START_BYTE
										0x07: BUS_OVER_RUN_ERROR
										0X08: MAX_LENGTH_ERROR
										0x09: MIN_LENGTH_ERROR
										0x0A: RECEIVE_CHARACTER_TIMEOUT
*/

#define ACK_READ_SUCCESS 	0xBB
#define ACK_READ_FAIL		0xEE

#define ACK_WRITE			0xEE
#define ACK_WRITE_SUCCESS	0x01

#define MAX_BYTES			0x80 // 128 Bytes

typedef struct
{
	float heading;
	float roll;
	float pitch;

	float gyro_x;
	float gyro_y;
	float gyro_z;

	float quaternion_w;
	float quaternion_x;
	float quaternion_y;
	float quaternion_z;

  float timestamp;
} imu_data;

enum write_ack_response
{
	WRITE_SUCCESS,
	WRITE_FAIL,
	WRITE_REGMAP_INVALID_ADDRESS,
	WRITE_REGMAP_WRITE_DISABLED,
	WRITE_WRONG_START_BYTE,
	WRITE_BUS_OVER_RUN_ERROR,
	WRITE_MAX_LENGTH_ERROR,
	WRITE_MIN_LENGTH_ERROR,
	WRITE_RECEIVE_CHARACTER_TIMEOUT
};

enum read_ack_response
{
	READ_SUCCESS,
	READ_FAIL,
	READ_REGMAP_INVALID_ADDRESS,
	READ_REGMAP_WRITE_DISABLED,
	READ_WRONG_START_BYTE,
	READ_BUS_OVER_RUN_ERROR,
	READ_MAX_LENGTH_ERROR,
	READ_MIN_LENGTH_ERROR,
	READ_RECEIVE_CHARACTER_TIMEOUT
};

enum BNO055_states
{
	RECEIVING_HEADER,
	RECEIVING_ERROR_CODE,
	RECEIVING_LENGTH,
	RECEIVING_DATA,
	RECEIVING_OPERATION_RESULT,
	ERROR,
	FINISHED
};

enum BNO055_operations
{
	READ_REGISTER,
	WRITE_REGISTER
};

void BNO055_init(void);

void BNO055_write_single_register(uint8_t register_address, uint8_t data);
void BNO055_write_register(uint8_t register_address, uint8_t length, uint8_t *data);
void BNO055_read_single_register(uint8_t register_address);
void BNO055_read_register(uint8_t register_address, uint8_t length);

void BNO055_interrupt_handler(uint8_t byte);

void BNO055_get_euler_angles(float *angles);
void BNO055_trigger_get_data(void);

int BNO055_received_new_data();

void BNO055_get_imu_data(imu_data *data);

#endif
