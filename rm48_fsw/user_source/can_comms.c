#include "can_comms.h"

static volatile float last_lidar_height;
static volatile float last_heading_reading;
static float last_att_roll, last_att_pitch, last_att_yaw;

static volatile int _new_height_msg = 0;

void get_sensor_msg(uint8_t* msg_data, sensor_msg m)
{
    int i = 0;
	
	union {
        float output[2];
        uint8_t input[8];
    } uint8_to_f32_2;

    union {
        int16_t output;
        uint8_t input[2];
    } uint8_to_i16;

	switch(m)
	{
		case HEIGHT_HEADING_MSG:

		    for(i=0; i<8; ++i)
		    {
		    	uint8_to_f32_2.input[i] = msg_data[i];
		    }

		    last_lidar_height = uint8_to_f32_2.output[0];
		    last_heading_reading = uint8_to_f32_2.output[1];
		    _new_height_msg = 1;
			break;
		case FLOW_MSG:
			break;
		case BNO_ATT_MSG:
			uint8_to_i16.input[0] = msg_data[0];
			uint8_to_i16.input[1] = msg_data[1];
			last_att_roll = (float)uint8_to_i16.output/(float)16.0f;

			uint8_to_i16.input[0] = msg_data[2];
			uint8_to_i16.input[1] = msg_data[3];
			last_att_pitch = (float)uint8_to_i16.output/(float)16.0f;

			uint8_to_i16.input[0] = msg_data[4];
			uint8_to_i16.input[1] = msg_data[5];
			last_att_yaw = (float)uint8_to_i16.output/(float)16.0f;
			break;
	}
}

float get_last_can_height_msg(void)
{
	return last_lidar_height;
}

float get_last_can_heading_msg(void)
{
	return last_heading_reading;
}

int new_height_avail(void)
{
	int tmp = _new_height_msg;
	if(tmp)
	{
		_new_height_msg = 0;
	}
	return tmp;
}

void get_last_attitude(float *att)
{
	att[0] = last_att_roll;
	att[1] = last_att_pitch;
	att[2] = last_att_yaw;
}

void publish_roll_pitch(float roll, float pitch)
{
	union {
		float input[2];
		uint8_t output[8];
	} conv_f32_2_to_u8;

	conv_f32_2_to_u8.input[0] = roll;
	conv_f32_2_to_u8.input[1] = pitch;

	canTransmit(canREG3, canMESSAGE_BOX3, conv_f32_2_to_u8.output);
}

void publish_yaw_height_estimate(float yaw, float height_estimate)
{
	union {
		float input[2];
		uint8_t output[8];
	} conv_f32_2_to_u8;

	conv_f32_2_to_u8.input[0] = yaw;
	conv_f32_2_to_u8.input[1] = height_estimate;

	canTransmit(canREG3, canMESSAGE_BOX4, conv_f32_2_to_u8.output);	
}

void publish_vert_velocity_estimate(float v)
{
	union {
		float input[2];
		uint8_t output[8];
	} conv_f32_2_to_u8;

	conv_f32_2_to_u8.input[0] = v;
	conv_f32_2_to_u8.input[1] = 0.0f;

	canTransmit(canREG3, canMESSAGE_BOX5, conv_f32_2_to_u8.output);
}

void publish_motor_commands(float *cmds)
{
	union {
		uint16_t input[4];
		uint8_t output[8];
	} conv_ui16_4_to_u8;	

	conv_ui16_4_to_u8.input[0] = (uint16_t)(cmds[0]*(float)65535.0f);
	conv_ui16_4_to_u8.input[1] = (uint16_t)(cmds[1]*(float)65535.0f);
	conv_ui16_4_to_u8.input[2] = (uint16_t)(cmds[2]*(float)65535.0f);
	conv_ui16_4_to_u8.input[3] = (uint16_t)(cmds[3]*(float)65535.0f);

	canTransmit(canREG3, canMESSAGE_BOX6, conv_ui16_4_to_u8.output);
}