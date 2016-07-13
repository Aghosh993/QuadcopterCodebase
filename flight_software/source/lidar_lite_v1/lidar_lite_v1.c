#include "lidar_lite_v1.h"

void lidar_lite_initialize_instance(lidar_lite_sensor_instance *inst)
{
	inst->sensor_measurement_state = STATE_IDLE;
	inst->last_measured_distance = 0.0f;
}

void lidar_lite_propagate_state_machine(lidar_lite_sensor_instance *inst)
{
	uint16_t raw_measurement = 0U;

	switch(inst->sensor_measurement_state)
	{
		case STATE_IDLE:
			#ifndef USE_TIMEOUTS
				lidar_lite_trigger_measurement();
				inst->sensor_measurement_state = STATE_INITIATED_MEASUREMENT;
			#endif
			#ifdef USE_TIMEOUTS
				if(lidar_lite_trigger_measurement() < 0)
				{
					inst->sensor_measurement_state = STATE_TRIGGER_TIMEOUT;
				}
				else
				{
					inst->sensor_measurement_state = STATE_INITIATED_MEASUREMENT;
				}
			#endif
			break;
		case STATE_INITIATED_MEASUREMENT:
			#ifndef USE_TIMEOUTS
				raw_measurement = lidar_lite_get_raw_data();
				inst->last_measured_distance = (float)raw_measurement/(float)100;
				inst->sensor_measurement_state = STATE_IDLE;
			#endif
			#ifdef USE_TIMEOUTS
				raw_measurement = lidar_lite_get_raw_data();
				if(raw_measurement == 0xFFFF)
				{
					inst->sensor_measurement_state = STATE_MEASUREMENT_TIMEOUT;
				}
				else
				{
					inst->last_measured_distance = (float)raw_measurement/(float)100;
					inst->sensor_measurement_state = STATE_IDLE;
				}
			#endif
			break;
		#ifdef USE_TIMEOUTS
		case STATE_TRIGGER_TIMEOUT:
			lidar_lite_reset_data_bus();
			inst->sensor_measurement_state = STATE_IDLE;
			break;
		case STATE_MEASUREMENT_TIMEOUT:
			lidar_lite_reset_data_bus();
			inst->sensor_measurement_state = STATE_IDLE;
			break;
		#endif
	}
}

float lidar_lite_get_latest_measurement(lidar_lite_sensor_instance inst)
{
	return inst.last_measured_distance;
}

int lidar_lite_trigger_measurement(void)
{
	#ifndef USE_TIMEOUTS
		lidar_lite_i2c_write_byte(LIDAR_LITE_COMMAND_CONTROL_REG, LIDAR_LITE_DC_COMP_CONV_MASK);
		return 0;
	#endif
	#ifdef USE_TIMEOUTS
		if(lidar_lite_i2c_write_byte_with_timeout(LIDAR_LITE_COMMAND_CONTROL_REG, LIDAR_LITE_DC_COMP_CONV_MASK) != WRITE_SUCCESS)
		{
			return -1;
		}
		return 0;
	#endif
}

uint16_t lidar_lite_get_raw_data(void)
{
	uint8_t raw_height_data[2U];
	#ifndef USE_TIMEOUTS
		lidar_lite_i2c_read_data(LIDAR_LITE_HEIGHT_DATA_START_REG, 2U, raw_height_data);
		return raw_height_data[0U] << 8 | raw_height_data[1U];
	#endif
	#ifdef USE_TIMEOUTS
		if(lidar_lite_i2c_read_data_with_timeout(LIDAR_LITE_HEIGHT_DATA_START_REG, 2U, raw_height_data) == READ_SUCCESS)
		{
			return raw_height_data[0U] << 8 | raw_height_data[1U];
		}
		return 0xFFFF;		
	#endif
}

void lidar_lite_reset_data_bus(void)
{
	lidar_lite_i2c_reset();
	lidar_lite_i2c_bus_setup();
}