#include <bno055_reader.h>
#include "mission_timekeeper.h"

static volatile enum BNO055_states state;
static volatile enum BNO055_operations operation;
static volatile uint8_t packet_buffer[MAX_BYTES];
static int _received_new_data = 0;
static imu_data _imu_data;

extern serialport *tm4c_port2_ptr;

static int bno055_triggered_waiting;

void BNO055_process_buffer(void);

/*
	This function relies on interrupts being enabled for blocking timekeeper_delay() to work!!
*/

void BNO055_init(void)
{
	bno055_triggered_waiting = 0;
	BNO055_write_single_register(0x3d, 0x00);
	timekeeper_delay(20U);
	BNO055_write_single_register(0x3d, 0xC); // 0x8 = failsafe for IMU-only fusion mode
	timekeeper_delay(500U);

	// BNO055_trigger_get_data();
}

void BNO055_write_single_register(uint8_t register_address, uint8_t data)
{
	operation = WRITE_REGISTER;
	state = RECEIVING_HEADER;

	uint8_t tx_array[5];
	tx_array[0] = 0xAA;
	tx_array[1] = 0x00;
	tx_array[2] = register_address;
	tx_array[3] = 0x01;
	tx_array[4] = data;

	serialport_send_data_buffer_blocking(tm4c_port2_ptr, tx_array, 5);
}

void BNO055_write_register(uint8_t register_address, uint8_t length, uint8_t *data)
{
	operation = WRITE_REGISTER;
	state = RECEIVING_HEADER;

	uint8_t tx_array[4];
	tx_array[0] = 0xAA;
	tx_array[1] = 0x00;
	tx_array[2] = register_address;
	tx_array[3] = 0x01;

	serialport_send_data_buffer_blocking(tm4c_port2_ptr, tx_array, 4);
	serialport_send_data_buffer_blocking(tm4c_port2_ptr, data, length);
}

void BNO055_read_single_register(uint8_t register_address)
{
	operation = READ_REGISTER;
	state = RECEIVING_HEADER;

	uint8_t tx_array[4];
	tx_array[0] = 0xAA;
	tx_array[1] = 0x01;
	tx_array[2] = register_address;
	tx_array[3] = 0x01;

	serialport_send_data_buffer_blocking(tm4c_port2_ptr, tx_array, 4);
}

void BNO055_read_register(uint8_t register_address, uint8_t length)
{
	operation = READ_REGISTER;
	state = RECEIVING_HEADER;

	uint8_t tx_array[4];
	tx_array[0] = 0xAA;
	tx_array[1] = 0x01;
	tx_array[2] = register_address;
	tx_array[3] = length;

	serialport_send_data_buffer_blocking(tm4c_port2_ptr, tx_array, 4);
}

void BNO055_interrupt_handler(uint8_t byte)
{
	static uint8_t packet_length;
	static uint8_t i = 0;

	switch (operation)
	{
		case READ_REGISTER:
			switch (state)
			{
				case RECEIVING_HEADER:
					if (byte == ACK_READ_SUCCESS)
					{
						state = RECEIVING_LENGTH;
					}
					else
					{
						state = ERROR;
					}
					break;
				case RECEIVING_LENGTH:
					packet_length = byte;
					i = 0;
					state = RECEIVING_DATA;
					break;
				case RECEIVING_DATA:
					packet_buffer[i] = byte;
					i++;
					packet_length--;
					if (packet_length == 0)
					{
						state = FINISHED;
			            BNO055_process_buffer();
			            _received_new_data = 1;
			            bno055_triggered_waiting = 0;
					}
					break;
				case ERROR:
				default:
					break;
			}
			break;
		case WRITE_REGISTER:
			switch(state)
			{
				case RECEIVING_HEADER:
					if (byte == ACK_WRITE)
					{
						state = RECEIVING_OPERATION_RESULT;
					}
					else
					{
						state = ERROR;
					}
					break;
				case RECEIVING_OPERATION_RESULT:
					if (byte == ACK_WRITE_SUCCESS)
					{
						state = FINISHED;
					}
					else
					{
						state = ERROR;
					}
				case ERROR:
				default:
					break;
			}
			break;
	}
}

void BNO055_trigger_get_data(void)
{
	// if(bno055_triggered_waiting != 1)
	// {
		BNO055_read_register(0x14, 20);
		bno055_triggered_waiting = 1;
	// }	
}

void BNO055_process_buffer(void)
{
	union {
		uint8_t input[2];
		int16_t output;
	} bytes_to_signed_int16;


		/* Gyroscope x */
	bytes_to_signed_int16.input[0] = packet_buffer[0];
	bytes_to_signed_int16.input[1] = packet_buffer[1];

	_imu_data.gyro_x = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Gyroscope y */
	bytes_to_signed_int16.input[0] = packet_buffer[2];
	bytes_to_signed_int16.input[1] = packet_buffer[3];

	_imu_data.gyro_y = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Gyroscope z */
	bytes_to_signed_int16.input[0] = packet_buffer[4];
	bytes_to_signed_int16.input[1] = packet_buffer[5];

	_imu_data.gyro_z = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Heading */
	bytes_to_signed_int16.input[0] = packet_buffer[6];
	bytes_to_signed_int16.input[1] = packet_buffer[7];

	_imu_data.heading = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Roll */
	bytes_to_signed_int16.input[0] = packet_buffer[8];
	bytes_to_signed_int16.input[1] = packet_buffer[9];

	_imu_data.roll = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Pitch */
	bytes_to_signed_int16.input[0] = packet_buffer[10];
	bytes_to_signed_int16.input[1] = packet_buffer[11];

	_imu_data.pitch = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Quaternion w */
	bytes_to_signed_int16.input[0] = packet_buffer[6];
	bytes_to_signed_int16.input[1] = packet_buffer[7];

	_imu_data.quaternion_w = (float)bytes_to_signed_int16.output/(float)16384.0f;

	/* Quaternion x */
	bytes_to_signed_int16.input[0] = packet_buffer[8];
	bytes_to_signed_int16.input[1] = packet_buffer[9];

	_imu_data.quaternion_x = (float)bytes_to_signed_int16.output/(float)16384.0f;

	/* Quaternion y */
	bytes_to_signed_int16.input[0] = packet_buffer[10];
	bytes_to_signed_int16.input[1] = packet_buffer[11];

	_imu_data.quaternion_y = (float)bytes_to_signed_int16.output/(float)16384.0f;

	/* Quaternion z */
	bytes_to_signed_int16.input[0] = packet_buffer[10];
	bytes_to_signed_int16.input[1] = packet_buffer[11];

	_imu_data.quaternion_z = (float)bytes_to_signed_int16.output/(float)16384.0f;
}

int BNO055_received_new_data()
{
  int r = _received_new_data;
  _received_new_data = 0;
  return r;
}

void BNO055_get_imu_data(imu_data *data)
{
	data->roll = _imu_data.roll;
	data->pitch = _imu_data.pitch;
	data->heading = _imu_data.heading;

	data->gyro_x = _imu_data.gyro_x;
	data->gyro_y = _imu_data.gyro_y;
	data->gyro_z = _imu_data.gyro_z;

	data->quaternion_x = _imu_data.quaternion_x;
	data->quaternion_y = _imu_data.quaternion_y;
	data->quaternion_z = _imu_data.quaternion_z;
	data->quaternion_w = _imu_data.quaternion_w;
}
