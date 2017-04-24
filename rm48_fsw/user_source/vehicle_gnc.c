#include "vehicle_gnc.h"

static volatile  imu_scaled_data_struct imu_data;
static volatile filtered_quadrotor_state st_vector;

static uint8_t gnc_en;

// static float floating_pt_abs(float n)
// {
// 	if(n > 0.0f)
// 	{
// 		return n;
// 	}
// 	return n * -1.0f;
// }

// static float start_height;

void gnc_init(void)
{
	imu_hal_init(); // Initialize IMU I2C bus
	initialize_imu(SCALE_2G, SCALE_250_DPS, SCALE_1POINT3_GAUSS, &imu_data); // Set up IMU registers
	init_comp_filter(&st_vector);	// Initialize complementary filter data structs/vars
	do_bias_calculation(&imu_data); // Bias calibration for IMU (gyro+accel)
	controller_init_vars();			// Set controller gains
	// set_controller_mode(MODE_ANGULAR_POSITION_CONTROL); // Set controller to stabilize (i.e. angular control) mode
	set_controller_mode(MODE_ANGULAR_RATE_CONTROL); // Set controller to acro (i.e. angular rate control) mode
	enable_controller(); // Enable all PID loops
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

void gnc_get_vehicle_state(void)
{
	/*
		Read Gyro, accelerometer and magnetometer sensors,
		and scale data to engineering (i.e. SI) units.
	 */
	if(get_scaled_imu_data(&imu_data) >= 0)
	{
		/*
			Obtain filtered vehicle state through complementary filter.
		 */
		get_filtered_vehicle_state(&st_vector, &imu_data);
	}
}

void gnc_get_raw_sensor_data(gnc_raw_data *ret)
{
	ret->roll_gyro = imu_data.gyro_data[AXIS_ROLL];
	ret->pitch_gyro = imu_data.gyro_data[AXIS_PITCH];
	ret->yaw_gyro = imu_data.gyro_data[AXIS_YAW];

	ret->x_accel = imu_data.accel_data[AXIS_X];
	ret->y_accel = imu_data.accel_data[AXIS_Y];
	ret->z_accel = imu_data.accel_data[AXIS_Z];
}

void gnc_get_state_vector_data(gnc_state_data *ret)
{
	ret->roll = st_vector.roll;
	ret->pitch = st_vector.pitch;
	ret->yaw = st_vector.yaw;
}

float gnc_get_vertical_dynamic_acceleration(void)
{
	return st_vector.vertical_dynamic_acceleration_post_lpf;
}

// void velocity_controller_update(float vel_cmd_x, float vel_cmd_y, float vel_x, float vel_y, float *roll_cmd, float *pitch_cmd)
// {
// 	generate_attitude_commands(vel_x, vel_y, vel_cmd_x, vel_cmd_y, roll_cmd, pitch_cmd);
// }

void gnc_integral_disable(void)
{
	rate_controller_integral_disable();
	attitude_controller_integral_disable();
}

void gnc_integral_enable(void)
{
	rate_controller_integral_enable();
	attitude_controller_integral_enable();
}

void gnc_vehicle_stabilization_outerloop_update(float roll_cmd_in,
												float pitch_cmd_in,
												float yaw_cmd_in,
												float *roll_rate_cmd_out,
												float *pitch_rate_cmd_out,
												float *yaw_rate_cmd_out)
{
	generate_rate_commands(&st_vector, &imu_data, roll_cmd_in, pitch_cmd_in, yaw_cmd_in, roll_rate_cmd_out, pitch_rate_cmd_out, yaw_rate_cmd_out);
}

void gnc_vehicle_stabilization_innerloop_update(float roll_rate_cmd_in,
												float pitch_rate_cmd_in,
												float yaw_rate_cmd_in,
												float throttle_value_in,
												double *motor_commands_out)
{
	rate_controller_update(motor_commands_out, &imu_data, roll_rate_cmd_in, pitch_rate_cmd_in, yaw_rate_cmd_in, throttle_value_in);
}

void gnc_height_kalman_struct_init(height_kalman_data_struct *str, float filter_dt_sec, float r_lidar)
{
	str->filter_dt = filter_dt_sec;
	str->r_lidar = r_lidar;

	str->height_estimated = 0.0f;
	str->vertical_velocity_estimated = 0.0f;

	str->h_lidar_global_coords = 0.059f;
	str->a_accel_global_coords = 0.0f;

	/*Initialize the P matrix to essentially an identity matrix to be on the safe side...?*/
	str->p11 = 100.0f;
	str->p12 = 0.0f;
	str->p21 = 0.0f;
	str->p22 = 10.0f;

	str->g11 = 0.0f;
	str->g21 = 0.0f;
}

void gnc_height_kalman_update(height_kalman_data_struct *str, float lidar_height_measurement, float accelerometer_z_measurement, float vehicle_roll_deg, float vehicle_pitch_deg)
{
	/*Prediction intermediate variables:*/
	float p11_predicted, p12_predicted, p21_predicted, p22_predicted;
	float height_predicted, vertical_velocity_predicted;

	/*Predict:*/

	height_predicted = str->height_estimated + (str->vertical_velocity_estimated * str->filter_dt) + (0.50f * str->a_accel_global_coords * str->filter_dt * str->filter_dt);
	vertical_velocity_predicted = str->vertical_velocity_estimated + (str->a_accel_global_coords * str->filter_dt);

	p11_predicted = str->p11 + str->p21 * str->filter_dt + str->p12 * str->filter_dt + str->p22 * str->filter_dt * str->filter_dt;
	p12_predicted = str->p12 + str->p22 * str->filter_dt;
	p21_predicted = str->p21 + str->p22 * str->filter_dt;
	p22_predicted = str->p22;

	// Corariance prediction noise.. aka the Q matrix :)
	p11_predicted += 0.5f;
	p12_predicted += 0.5f;
	p21_predicted += 0.5f;
	p22_predicted += 0.5f;

	/*Update:*/
	float roll_cos = cos(vehicle_roll_deg * DEGREES_TO_RADIANS_CONVERSION_FACTOR);
	float pitch_cos = cos(vehicle_pitch_deg * DEGREES_TO_RADIANS_CONVERSION_FACTOR);
	float roll_pitch_cos_prod = roll_cos * pitch_cos;
	if(roll_pitch_cos_prod != 0.0f)
	{
		str->h_lidar_global_coords = lidar_height_measurement*(roll_cos * pitch_cos);
		str->a_accel_global_coords = (accelerometer_z_measurement/(roll_cos * pitch_cos))- 9.810f;	
	}

	str->g11 = p11_predicted/(p11_predicted + str->r_lidar);
	str->g21 = p21_predicted/(p11_predicted + str->r_lidar);

	// for an observer, we instead just overwrite g11 and g21:
	// str->g11 = 0.04f;
	// str->g21 = 0.10f;

	float innovation = str->h_lidar_global_coords - height_predicted;
	str->height_estimated = height_predicted + str->g11 * innovation;
	str->vertical_velocity_estimated = vertical_velocity_predicted + str->g21 * innovation;

	str->p11 = p11_predicted - p11_predicted * str->g11;
	str->p12 = p12_predicted - p12_predicted * str->g11;
	str->p21 = -1.0f * str->g21 * p11_predicted + p21_predicted;
	str->p22 = -1.0f * str->g21 * p12_predicted + p22_predicted;
}

float gnc_bno055_get_relative_heading(float raw_heading, float prior_heading)
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

float gnc_get_height_controller_throttle_command(float height_commanded, float height_estimate, float vertical_velocity_estimate)
{
	static float height_pid_err_accum = 0.0f;
	float height_error = 0.0f;
	float height_pid_adj = 0.0f;
	float height_pid_max_adj = 0.275f;

	// Compute height error in meters:
	height_error = height_commanded - height_estimate;
	// Compute height accumulated error in meter*seconds:
	height_pid_err_accum += height_error * HEIGHT_CONTROL_DT;

	// Saturate the height accumulated error integral:
	if(height_pid_err_accum < -1.0f)
	{
		height_pid_err_accum = -1.0f;
	}
	if(height_pid_err_accum > 1.0f)
	{
		height_pid_err_accum = 1.0f;
	}

	// Compute height PID adjustment:
	height_pid_adj = (1.17 * height_error) + (height_pid_err_accum * 0.00f) + (-0.72f * vertical_velocity_estimate);

	// Saturate height PID adjustment:
	if(height_pid_adj < -1.0f * height_pid_max_adj)
	{
		height_pid_adj = -1.0f * height_pid_max_adj;
	}
	if(height_pid_adj > height_pid_max_adj)
	{
		height_pid_adj = height_pid_max_adj;
	}

	float height_controller_throttle_cmd = gnc_height_controller_thrust_offset(0.3048f, height_estimate) + height_pid_adj;

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