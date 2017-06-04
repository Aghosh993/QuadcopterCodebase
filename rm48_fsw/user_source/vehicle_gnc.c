#include "vehicle_gnc.h"
#include "can_comms.h"

static volatile imu_scaled_data_struct imu_data;
static volatile complementary_filter_struct st_vector;
static volatile kalman_3state_struct h_state;
static volatile float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;
static volatile float motor1_cmd, motor2_cmd, motor3_cmd, motor4_cmd;
static volatile float att_kP, att_kD, att_rate_kP;

static volatile uint8_t gnc_en;

static void check_saturation(float *num, float lower, float upper)
{
	if(*num > upper)
	{
		*num = upper;
	}
	else if(*num < lower)
	{
		*num = lower;
	}
}

static void normalize_outputs(float* motor_cmds, float actuator_limit)
{
	int i = 0;
	float max_cmd = 0.0f;
	float scale_factor = 1.0f;

	for(i=0; i<4; ++i)
	{
		if(motor_cmds[i] < 0.0f)
		{
			motor_cmds[i] = 0.0f;
		}
		if(motor_cmds[i] > max_cmd)
		{
			max_cmd = motor_cmds[i];
		}
	}
	if(max_cmd > actuator_limit)
	{
		scale_factor = actuator_limit/max_cmd;
		for(i=0; i<4; ++i)
		{
			motor_cmds[i] *= scale_factor;
		}
	}
}

float gnc_height_controller_thrust_offset(float rotor_dia_meters, float height_meters)
{
	float min_thrust_offset = 0.49f;
	float max_thrust_offset = 0.56f;
	float slope = (float)(max_thrust_offset - min_thrust_offset)/(float)(rotor_dia_meters * 0.65f);
	if(height_meters >= 0.0f && height_meters <= rotor_dia_meters * (float)0.65f)
	{
		return min_thrust_offset + (slope * height_meters);
	}
	if(height_meters > rotor_dia_meters * (float)0.65f)
	{
		return max_thrust_offset;
	}
	return min_thrust_offset;
}

void gnc_init(float attitude_kP, float attitude_kD, float rate_kP, gnc_control_mode m)
{
	initialize_imu(SCALE_2G, SCALE_250_DPS, SCALE_1POINT3_GAUSS, &imu_data); // Set up IMU registers
	init_complementary_filter(&st_vector, &imu_data,
	                          0.0025f, 0.085f, 1.0f, MODE_2NDORDER_COMPFILTER);

	kalman_3state_init(&h_state, &imu_data, 0.0025f);

	att_kP = attitude_kP;
	att_kD = attitude_kD;
	att_rate_kP = rate_kP;

	roll_rate_cmd = 0.0f;
	pitch_rate_cmd = 0.0f;
	yaw_rate_cmd = 0.0f;

	motor1_cmd = 0.0f;
	motor2_cmd = 0.0f;
	motor3_cmd = 0.0f;
	motor4_cmd = 0.0f;
	
	gnc_disable();
}

void gnc_enable(void)
{
	gnc_en = 1U;
}

void gnc_disable(void)
{
	gnc_en = 0U;
}

uint8_t gnc_enabled(void)
{
	return gnc_en;
}

void gnc_update_vehicle_state(void)
{
	/*
		Get IMU data:
	 */
	get_scaled_imu_data(&imu_data);

	/*
		Perform second-order complementary filter update to get roll and pitch from latest IMU data:
	 */
	update_complementary_filter(&st_vector);

	/*
		Use latest IMU data to propagate height state vector:
	 */
	kalman_3state_accel_update(&h_state, 0.0025f);

	/*
		If a new LIDAR altimeter/rangefinder data item is available use that to propagate height state vector:
	 */
	if(new_height_avail())
	{
		kalman_3state_lidar_update(&h_state, get_last_can_height_msg(), 0.040f);
	}

	#if defined GNC_USE_ABS_HEADING
		/*
			Bit of a hack to put the BNO055 yaw data into the "yaw" field of the state vector.
			Ideally that field should be populated using the MPU-9250's magnetometer-derived yaw via EKF or Complementary Filter... :)
		 */
		st_vector.state_vector.yaw = degrees_to_radians(get_last_can_heading_msg());
	#else
		st_vector.state_vector.yaw = 0.0f;
	#endif
}

void gnc_get_height_state_vector(float* h, float* v, float* a)
{
	*h = h_state.state_vector.z;
	*v = h_state.state_vector.v_z;
	*a = h_state.state_vector.a_z;
}

void gnc_get_raw_sensor_data(observation *ret)
{
	memcpy(ret, &(st_vector.sensor_data), sizeof(observation));
}

void gnc_get_state_vector_data(complementary_filter_struct *ret)
{
	memcpy(ret, &st_vector, sizeof(complementary_filter_struct));
}

void gnc_attitude_controller_update(float roll_cmd_in, float pitch_cmd_in, float yaw_rate_cmd_in)
{
	static float integrated_yaw_cmd = 0.0f;

	if(gnc_enabled())
	{
		if(yaw_rate_cmd_in < -1.0f*YAW_RATE_DEADBAND || yaw_rate_cmd_in > YAW_RATE_DEADBAND)
		{
			integrated_yaw_cmd += yaw_rate_cmd_in*-1.0f*YAW_RATE_INTEGRATION_MULTIPLIER;
		}
		if(integrated_yaw_cmd < -2.0f*(float)M_PI)
		{
			integrated_yaw_cmd = -2.0f*(float)M_PI;
		}
		else if(integrated_yaw_cmd > 2.0f*(float)M_PI)
		{
			integrated_yaw_cmd = 2.0f*(float)M_PI;
		}

		roll_rate_cmd = att_kP*(roll_cmd_in*ATTITUDE_CMD_MULTIPLIER - st_vector.state_vector.roll) - att_kD*st_vector.state_vector.roll_rate;
		pitch_rate_cmd = att_kP*(pitch_cmd_in*ATTITUDE_CMD_MULTIPLIER*-1.0f - st_vector.state_vector.pitch) - att_kD*st_vector.state_vector.pitch_rate;
		#if !defined GNC_USE_ABS_HEADING
			yaw_rate_cmd = att_kP*(yaw_rate_cmd_in*ATTITUDE_CMD_MULTIPLIER*-1.0f);
		#else
			yaw_rate_cmd = 0.65f*gnc_compass_get_relative_heading_rad(integrated_yaw_cmd, st_vector.state_vector.yaw);
		#endif
	}
}

void gnc_attitude_rate_controller_update(float throttle_value_in)
{
	float roll_adj = att_rate_kP*(roll_rate_cmd-degrees_to_radians(st_vector.imu_data->gyro_data[0]));
	float pitch_adj = att_rate_kP*(pitch_rate_cmd-degrees_to_radians(st_vector.imu_data->gyro_data[1]));
	float yaw_adj = att_rate_kP*2.0f*(yaw_rate_cmd-degrees_to_radians(st_vector.imu_data->gyro_data[2]));

	if(gnc_enabled())
	{
		motor1_cmd = throttle_value_in + roll_adj*0.5f + pitch_adj*0.5f + yaw_adj;
		motor2_cmd = throttle_value_in - roll_adj*0.5f + pitch_adj*0.5f - yaw_adj;
		motor3_cmd = throttle_value_in - roll_adj*0.5f - pitch_adj*0.5f + yaw_adj;
		motor4_cmd = throttle_value_in + roll_adj*0.5f - pitch_adj*0.5f - yaw_adj;
	}
}

void gnc_get_actuator_commands(float* commands)
{
	check_saturation(&motor1_cmd, ACTUATOR_LOW_LIM, ACTUATOR_HIGH_LIM);
	check_saturation(&motor2_cmd, ACTUATOR_LOW_LIM, ACTUATOR_HIGH_LIM);
	check_saturation(&motor3_cmd, ACTUATOR_LOW_LIM, ACTUATOR_HIGH_LIM);
	check_saturation(&motor4_cmd, ACTUATOR_LOW_LIM, ACTUATOR_HIGH_LIM);

	if(gnc_enabled())
	{
		commands[0] = motor1_cmd;
		commands[1] = motor2_cmd;
		commands[2] = motor3_cmd;
		commands[3] = motor4_cmd;
	}
	else
	{
		commands[0] = 0.0f;
		commands[1] = 0.0f;
		commands[2] = 0.0f;
		commands[3] = 0.0f;
	}
}

float gnc_compass_get_relative_heading(float raw_heading, float prior_heading)
{
	float delta_theta = raw_heading - prior_heading;
	if(delta_theta > 180.0f)
	{
		delta_theta -= 360.0f;
	}
	if(delta_theta < -180.0f)
	{
		delta_theta += 360.0f;
	}
	return delta_theta;
}

float gnc_compass_get_relative_heading_rad(float raw_heading, float prior_heading)
{
	float delta_theta = raw_heading - prior_heading;
	if(delta_theta > 3.14159f)
	{
		delta_theta -= 2.0f*3.14159f;
	}
	if(delta_theta < -3.14159f)
	{
		delta_theta += 2.0f*3.14159f;
	}
	return delta_theta;
}

float gnc_get_height_controller_throttle_command(float height_commanded)
{
	float height_error = 0.0f;
	float height_pid_adj = 0.0f;
	float height_pid_max_adj = 0.275f;

	// Compute height error in meters:
	height_error = height_commanded - h_state.state_vector.z;

	// Compute height PID adjustment:
	height_pid_adj = (1.17 * height_error) + (-0.72f * h_state.state_vector.v_z);

	// Saturate height PID adjustment:
	if(height_pid_adj < -1.0f * height_pid_max_adj)
	{
		height_pid_adj = -1.0f * height_pid_max_adj;
	}
	if(height_pid_adj > height_pid_max_adj)
	{
		height_pid_adj = height_pid_max_adj;
	}

	float height_controller_throttle_cmd = gnc_height_controller_thrust_offset(0.3048f, h_state.state_vector.z) + height_pid_adj;

	return height_controller_throttle_cmd;
}

// float get_compensated_sf10_data(vehicle_relative_height_tracker *tr, 
// 											float sf10_raw_measurement, float sf10_previous_raw_measurement,
// 											float current_height_cmd, float previous_height_cmd)
// {
// 	switch(*tr)
// 	{
// 		// NEED TO ADDRESS POTENTIAL UNDEFINED CASES!!!
// 		case STATE_VEHICLE_ABOVE_GROUND:
// 			if(sf10_raw_measurement - sf10_previous_raw_measurement < -1.0f * SF10_UGV_THRESHOLD &&
// 					floating_pt_abs(current_height_cmd - previous_height_cmd) < SF10_UGV_THRESHOLD)
// 			{
// 				*tr = STATE_TRANSITIONING_TO_UGV;
// 				start_height = sf10_previous_raw_measurement;
// 				return sf10_raw_measurement;
// 			}
// 			return sf10_raw_measurement;
// 			break;
// 		case STATE_TRANSITIONING_TO_UGV:
// 			if(sf10_raw_measurement - start_height < -1.0f * SF10_UGV_THRESHOLD &&
// 					floating_pt_abs(current_height_cmd - previous_height_cmd) < SF10_UGV_THRESHOLD)
// 			{
// 				*tr = STATE_VEHICLE_ABOVE_UGV;
// 				return sf10_raw_measurement + UGV_HEIGHT;
// 			}
// 			else
// 			{
// 				*tr = STATE_VEHICLE_ABOVE_GROUND;
// 				return sf10_raw_measurement;
// 			}
// 			break;
// 		case STATE_VEHICLE_ABOVE_UGV:
// 			if(sf10_raw_measurement - sf10_previous_raw_measurement > SF10_UGV_THRESHOLD &&
// 					floating_pt_abs(current_height_cmd - previous_height_cmd) < SF10_UGV_THRESHOLD)
// 			{
// 				*tr = STATE_TRANSITIONING_TO_GROUND;
// 				start_height = sf10_previous_raw_measurement;
// 				return sf10_raw_measurement + UGV_HEIGHT;
// 			}
// 			return sf10_raw_measurement + UGV_HEIGHT;
// 			break;
// 		case STATE_TRANSITIONING_TO_GROUND:
// 			if(sf10_raw_measurement - start_height > SF10_UGV_THRESHOLD &&
// 					floating_pt_abs(current_height_cmd - previous_height_cmd) < SF10_UGV_THRESHOLD)
// 			{
// 				*tr = STATE_VEHICLE_ABOVE_GROUND;
// 				return sf10_raw_measurement;
// 			}
// 			else
// 			{
// 				*tr = STATE_VEHICLE_ABOVE_UGV;
// 				return sf10_raw_measurement + UGV_HEIGHT;
// 			}
// 			break;
// 		default:
// 			return sf10_raw_measurement;
// 			break;
// 	}
// 	return sf10_raw_measurement;
// }

// void automated_landing_sequence(float relative_time, float *height_setpoint_output, float actual_height)
// {
// 	float h_setpoint = RAPID_DESCENT_INITIAL_HEIGHT - RAPID_DESCENT_VELOCITY * relative_time;
// 	if(actual_height < RAPID_DESCENT_MOTOR_CUTOFF_HEIGHT)
// 	{
// 		*height_setpoint_output = 0.0f;

// 		// We're done!! Disarm all systems and await shutoff/reset...
// 		disable_controller();
// 		QuadRotor_motor1_stop();
// 		QuadRotor_motor2_stop();
// 		QuadRotor_motor3_stop();
// 		QuadRotor_motor4_stop();
// 		while(1);
// 	}
// 	else
// 	{
// 		*height_setpoint_output = h_setpoint;
// 	}
// }

// void estimate_lateral_velocity(float* velocity_x_output, float *velocity_y_output, pxflow_flow_data_struct flow_data_input, float height_input, float current_roll_degrees, float current_pitch_degrees)
// {
// 	static float last_roll_angle = 0.0f;
// 	static float last_pitch_angle = 0.0f;

// 	float d_roll = current_roll_degrees * DEGREES_TO_RADIANS_CONVERSION_FACTOR - last_roll_angle;
// 	float d_pitch = current_pitch_degrees * DEGREES_TO_RADIANS_CONVERSION_FACTOR - last_pitch_angle;

// 	float k1 = 1.0f;
// 	float k2 = 1.0f;

// 	// *velocity_x_output = k1 * ((float)flow_data_input.raw_x_flow * 3.940f * height_input - k2 * d_roll * 1.0f * height_input);
// 	// *velocity_y_output = k1 * ((float)flow_data_input.raw_y_flow * -3.940f * height_input - k2 * d_pitch * -1.0f * height_input);
// 	*velocity_x_output = flow_data_input.x_velocity;
// 	*velocity_y_output = flow_data_input.y_velocity;

// 	last_roll_angle = current_roll_degrees * DEGREES_TO_RADIANS_CONVERSION_FACTOR;
// 	last_pitch_angle = current_pitch_degrees * DEGREES_TO_RADIANS_CONVERSION_FACTOR;
// }

// void px4flow_get_bias(int16_t* bias_x, int16_t* bias_y)
// {
// 	int i = 0;

// 	int accum_x, accum_y;

// 	accum_x = 0;
// 	accum_y = 0;

// 	pxflow_flow_data_struct st;

// 	for(i = 0; i < 1000; ++i)
// 	{
// 		while(!px4flow_received_new_data());
// 		get_pxflow_flow_data(&st);
// 		accum_x += (int)st.raw_x_flow;
// 		accum_y += (int)st.raw_y_flow;
// 		// printf("%d %d\r\n", st.raw_x_flow, st.raw_y_flow);
// 	}
// 	*bias_x = (int16_t)((float)accum_x/(float)1000);
// 	*bias_y = (int16_t)((float)accum_y/(float)1000);
// }