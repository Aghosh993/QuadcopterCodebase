/*
 * File: pid_controller.c
 *
 * Author: Abhimanyu Ghosh
 * 			Controls and Robotics Research Laboratory (CRRL)
 * 			NYU Polytechnic School of Engineering
 * 			(c) 2014-2015
 */

/*
 * Algorithmic includes:
 */

#include "vehicle_pid_controller.h"

static volatile pid_data_struct roll_rate_controller;
static volatile pid_data_struct pitch_rate_controller;
static volatile pid_data_struct yaw_rate_controller;

#ifndef ATTITUDE_CONTROL_USE_LPF

	static volatile pid_data_struct roll_controller;
	static volatile pid_data_struct pitch_controller;

#endif

#ifdef ATTITUDE_CONTROL_USE_LPF

	static volatile pi_d_lpf_data_struct roll_controller;
	static volatile pi_d_lpf_data_struct pitch_controller;

#endif

static volatile pid_data_struct velocity_x_controller;
static volatile pid_data_struct velocity_y_controller;

#ifdef NESTED_HEIGHT_CONTROLLER
static volatile pid_data_struct vertical_acceleration_controller;
static volatile pid_data_struct height_controller;
#endif

#ifdef SINGLE_STAGE_HEIGHT_CONTROLLER
static volatile pid_data_struct single_stage_height_controller;
static volatile pi_d_lpf_data_struct single_stage_height_controller_lpf;
#endif

static volatile controller_mode control_mode;

volatile double roll_cmd_multiplier, pitch_cmd_multiplier, yaw_cmd_multiplier,
		roll_rate_cmd_multiplier, pitch_rate_cmd_multiplier, yaw_rate_cmd_multiplier,
		height_cmd_multiplier, vertical_velocity_cmd_multiplier;

static volatile uint8_t controller_en;

void enable_controller(void)
{
	controller_en = 1U;
	enable_pid_controller(&roll_rate_controller);
	enable_pid_controller(&pitch_rate_controller);
}

void disable_controller(void)
{
	controller_en = 0U;
	disable_pid_controller(&roll_rate_controller);
	disable_pid_controller(&pitch_rate_controller);
}

uint8_t get_controller_status(void)
{
	return controller_en;
}

void set_controller_mode(controller_mode mode)
{
	control_mode = mode;
}

void controller_init_vars(void)
{
	controller_en = 0.0f;
	control_mode = MODE_ANGULAR_POSITION_CONTROL;

	/*
	 * Limit Roll and Pitch to +/- 20 degrees
	 * Limit Yaw to +/- 180 degrees
	 *
	 * Limit all angular rates to +/- 20 deg/s
	 *
	 * (Angular) Units below and elsewhere are in Radians:
	 */

	roll_cmd_multiplier = 2.0f*0.349065556f;
	pitch_cmd_multiplier = -2.0f*0.349065556f;
	yaw_cmd_multiplier = PI_VAL;

	roll_rate_cmd_multiplier = 2.0f*0.349065556f;
	pitch_rate_cmd_multiplier = -2.0f*0.349065556f;
	yaw_rate_cmd_multiplier = 1.250f;//0.785f;//0.349065556f;

	height_cmd_multiplier = MAX_HEIGHT;
	vertical_velocity_cmd_multiplier = 1.250f;

#ifdef PROP_PITCH_3DOT8_DEGREES
	init_pid_controller(&roll_rate_controller, roll_rate_kP, roll_rate_kI, roll_rate_kD,
												ROLL_RATE_PID_DT, ROLL_RATE_ACCUM_ERR_ABS_MAX, ROLL_RATE_MAX_ABS_ADJUSTMENT);
	init_pid_controller(&pitch_rate_controller, pitch_rate_kP, pitch_rate_kI, pitch_rate_kD,
												PITCH_RATE_PID_DT, PITCH_RATE_ACCUM_ERR_ABS_MAX, PITCH_RATE_MAX_ABS_ADJUSTMENT);
	init_pid_controller(&yaw_rate_controller, yaw_rate_kP, yaw_rate_kI, yaw_rate_kD,
												YAW_RATE_PID_DT, YAW_RATE_ACCUM_ERR_ABS_MAX, YAW_RATE_MAX_ABS_ADJUSTMENT);

	#ifndef ATTITUDE_CONTROL_USE_LPF
		init_pid_controller(&roll_controller, roll_kP, roll_kI, roll_kD, //2.9f, 0.095f, 0.0005f,
												ROLL_PID_DT, ROLL_ACCUM_ERR_ABS_MAX, ROLL_MAX_ABS_ADJUSTMENT);
		init_pid_controller(&pitch_controller, pitch_kP, pitch_kI, pitch_kD,
												PITCH_PID_DT, PITCH_ACCUM_ERR_ABS_MAX, PITCH_MAX_ABS_ADJUSTMENT);
	#endif
	#ifdef ATTITUDE_CONTROL_USE_LPF
		init_pi_d_lpf_controller(&roll_controller, roll_kP, roll_kI, roll_kD, //2.9f, 0.095f, 0.0005f,
												ROLL_PID_DT, ROLL_ACCUM_ERR_ABS_MAX, ROLL_MAX_ABS_ADJUSTMENT,
												ROLL_PID_DERIV_LPF_FREQ);
		init_pi_d_lpf_controller(&pitch_controller, pitch_kP, pitch_kI, pitch_kD,
												PITCH_PID_DT, PITCH_ACCUM_ERR_ABS_MAX, PITCH_MAX_ABS_ADJUSTMENT,
												PITCH_PID_DERIV_LPF_FREQ);
	#endif
#endif

#ifdef NESTED_HEIGHT_CONTROLLER
	init_pid_controller(&vertical_acceleration_controller, 0.400f, 0.015f, 0.0f,
														0.010f, 0.4f, 0.30f);
	init_pid_controller(&height_controller, 0.50f, 0.15f, 0.00f,
												0.035f, 2.00f, 9.810f*0.50f);
#endif

#ifdef SINGLE_STAGE_HEIGHT_CONTROLLER
	// init_pid_controller(&single_stage_height_controller, 0.4f, 0.0f, 0.00f,
	// 							HEIGHT_CONTROL_DT, 0.75f, 0.285f);
	init_pi_d_lpf_controller(&single_stage_height_controller_lpf, 1.0f, 0.0f, 0.3f,
																	HEIGHT_CONTROL_DT, 0.75f, 0.285f,
																	4.0f);
#endif

	init_pid_controller(&velocity_x_controller, 0.90f, 0.85f, 0.0f,
													LATERAL_VELOCITY_CONTROL_DT, 1.5f, 0.70f);
	init_pid_controller(&velocity_y_controller, 0.90f, 0.85f, 0.0f,
													LATERAL_VELOCITY_CONTROL_DT, 1.5f, 0.70f);

	pid_inhibit_integral(&roll_controller);
	pid_inhibit_integral(&pitch_controller);

	pid_inhibit_integral(&roll_rate_controller);
	pid_inhibit_integral(&pitch_rate_controller);
	pid_inhibit_integral(&yaw_rate_controller);
}

void check_output_saturation(double* motor_output_buffer)
{
	uint8_t i = 0U;
	for(i=0;i<4;++i)
	{
		if(motor_output_buffer[i] < MOTOR_MIN_CMD)
		{
			motor_output_buffer[i] = MOTOR_MIN_CMD;
		}
		else if (motor_output_buffer[i] > MOTOR_MAX_CMD)
		{
			motor_output_buffer[i] = MOTOR_MAX_CMD;
		}
	}
}
#ifdef NESTED_HEIGHT_CONTROLLER
void generate_vertical_acceleration_commands(float actual_height, float target_height, float* vertical_acceleration_output)
{
	*vertical_acceleration_output = -1.0f * calculate_pid_adjustment(&height_controller, -1.0f*actual_height, -1.0f*target_height*height_cmd_multiplier);
}

void generate_thrust_commands(float actual_vertical_acceleration, float target_vertical_acceleration, float* thrust_output)
{
	*thrust_output = OPENLOOP_MOTOR_MIN_THRUST - calculate_pid_adjustment(&vertical_acceleration_controller, -1.0f*actual_vertical_acceleration, -1.0f*target_vertical_acceleration);
}
#endif

#ifdef SINGLE_STAGE_HEIGHT_CONTROLLER
void generate_thrust_commands(float actual_height, float target_height, float* throttle_command)
{
	// *throttle_command = OPENLOOP_MOTOR_MIN_THRUST - calculate_pid_adjustment(&single_stage_height_controller, -1.0f*actual_height, -1.0f*target_height*height_cmd_multiplier);
	*throttle_command = OPENLOOP_MOTOR_MIN_THRUST - calculate_pi_d_lpf_adjustment(&single_stage_height_controller_lpf, -1.0f*actual_height, -1.0f*target_height*height_cmd_multiplier);
}
#endif

void generate_attitude_commands(float velocity_x, float velocity_y, float target_velocity_x, float target_velocity_y, float* roll_output, float* pitch_output)
{
	*roll_output = calculate_pid_adjustment(&velocity_x_controller, velocity_x, target_velocity_x);
	*pitch_output = calculate_pid_adjustment(&velocity_y_controller, velocity_y, target_velocity_y);
}

static float bno055_get_relative_heading_rad(float raw_heading, float prior_heading)
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

void generate_rate_commands(filtered_quadrotor_state* ap_st, imu_scaled_data_struct* data, float joy_x, float joy_y, float joy_yaw, float* roll_rate_output, float* pitch_rate_output, float* yaw_rate_output)
{

	float roll, pitch;

	float deg_to_rad_conv_factor = (float)PI_VAL/(float)180.0f;

	// Convert to radians:
	roll = ap_st->roll*deg_to_rad_conv_factor;
	pitch = ap_st->pitch*deg_to_rad_conv_factor;

	switch(control_mode)
	{
	case MODE_ANGULAR_RATE_CONTROL:
		*pitch_rate_output = joy_y * pitch_rate_cmd_multiplier;
		*roll_rate_output = joy_x * roll_rate_cmd_multiplier;
		break;
	case MODE_ANGULAR_POSITION_CONTROL:
	#ifndef ATTITUDE_CONTROL_USE_LPF
		*roll_rate_output =  calculate_pid_adjustment(&roll_controller, roll, joy_x*roll_cmd_multiplier);
		*pitch_rate_output = calculate_pid_adjustment(&pitch_controller, pitch, joy_y*pitch_cmd_multiplier);
	#endif

	#ifdef ATTITUDE_CONTROL_USE_LPF
		*roll_rate_output =  calculate_pi_d_lpf_adjustment(&roll_controller, roll, joy_x*roll_cmd_multiplier);
		*pitch_rate_output = calculate_pi_d_lpf_adjustment(&pitch_controller, pitch, joy_y*pitch_cmd_multiplier);
	#endif
		break;
	}

	/*
	 * Manual yaw rate damping:
	 * At the moment we only let the user determine yaw rate command manually via joystick:
	 */

	*yaw_rate_output = -1.0f * joy_yaw * yaw_rate_cmd_multiplier;

	/*
	 * Closed-loop yaw control using full state vector:
	 */
	// *yaw_rate_output = 0.25f * bno055_get_relative_heading_rad((ap_st->yaw * deg_to_rad_conv_factor), joy_yaw) + -0.20f * deg_to_rad_conv_factor * data->gyro_data[AXIS_YAW];
	 // *yaw_rate_output = 0.20f * bno055_get_relative_heading_rad(joy_yaw, (ap_st->yaw * deg_to_rad_conv_factor));
}

void rate_controller_update(double * c_props, imu_scaled_data_struct* data, float roll_rate_cmd, float pitch_rate_cmd, float yaw_rate_cmd, float throttle_openloop_commanded)
{
	double motor_outputs[4];

	float corrected_gyro_data[3];
	get_corrected_scaled_gyro_data(data, corrected_gyro_data);

	float roll_rate_raw = (float)corrected_gyro_data[AXIS_ROLL]*(float)PI_VAL/(float)180.0;
	float pitch_rate_raw = (float)corrected_gyro_data[AXIS_PITCH]*(float)PI_VAL/(float)180.0;
	float yaw_rate_raw = (float)corrected_gyro_data[AXIS_YAW]*(float)PI_VAL/(float)180.0;

	// Initially command all 4 motors to neutral for data safety:

	motor_outputs[MOTOR_1] = 0.0f;
	motor_outputs[MOTOR_2] = 0.0f;
	motor_outputs[MOTOR_3] = 0.0f;
	motor_outputs[MOTOR_4] = 0.0f;

	// Compute PID adjustments:

	float roll_rate_net_adjustment = calculate_pid_adjustment(&roll_rate_controller, roll_rate_raw, roll_rate_cmd);
	float pitch_rate_net_adjustment = calculate_pid_adjustment(&pitch_rate_controller, pitch_rate_raw, pitch_rate_cmd);
	float yaw_rate_net_adjustment = calculate_pid_adjustment(&yaw_rate_controller, yaw_rate_raw, yaw_rate_cmd);

#ifdef MOTOR_1_EN
	motor_outputs[MOTOR_1] = throttle_openloop_commanded
							+(0.5f*roll_rate_net_adjustment)
							+(0.5f*pitch_rate_net_adjustment)
					#ifdef YAW_RATE_CLOSED_LOOP_ENABLED
							+ (double)yaw_rate_net_adjustment;
					#endif
					#ifndef YAW_RATE_CLOSED_LOOP_ENABLED
							- (double)yaw_rate_cmd;
					#endif
#endif

#ifdef MOTOR_2_EN
	motor_outputs[MOTOR_2] = throttle_openloop_commanded
							-(0.5f*roll_rate_net_adjustment)
							+(0.5f*pitch_rate_net_adjustment)
					#ifdef YAW_RATE_CLOSED_LOOP_ENABLED
							- (double)yaw_rate_net_adjustment;
					#endif
					#ifndef YAW_RATE_CLOSED_LOOP_ENABLED
							+ (double)yaw_rate_cmd;
					#endif
#endif

#ifdef MOTOR_3_EN
	motor_outputs[MOTOR_3] = throttle_openloop_commanded
							-(0.5f*roll_rate_net_adjustment)
							-(0.5f*pitch_rate_net_adjustment)
					#ifdef YAW_RATE_CLOSED_LOOP_ENABLED
							+ (double)yaw_rate_net_adjustment;
					#endif
					#ifndef YAW_RATE_CLOSED_LOOP_ENABLED
							- (double)yaw_rate_cmd;
					#endif
#endif

#ifdef MOTOR_4_EN
	motor_outputs[MOTOR_4] = throttle_openloop_commanded
							+(0.5f*roll_rate_net_adjustment)
							-(0.5f*pitch_rate_net_adjustment)
					#ifdef YAW_RATE_CLOSED_LOOP_ENABLED
							- (double)yaw_rate_net_adjustment;
					#endif
					#ifndef YAW_RATE_CLOSED_LOOP_ENABLED
							+ (double)yaw_rate_cmd;
					#endif
#endif

	check_output_saturation(motor_outputs);

	uint8_t i = 0U;
	if(controller_en == 1U)
	{
		for(i=0;i<4;++i)
		{
			c_props[i] = motor_outputs[i];
		}
	}
	else
	{
		for(i=0;i<4;++i)
		{
			c_props[i] = MOTOR_MIN_CMD;
		}
	}
}

void rate_controller_integral_disable()
{
	pid_inhibit_integral(&roll_rate_controller);
	pid_inhibit_integral(&pitch_rate_controller);
	pid_inhibit_integral(&yaw_rate_controller);
}
void attitude_controller_integral_disable()
{
	pid_inhibit_integral(&roll_controller);
	pid_inhibit_integral(&pitch_controller);
}

void rate_controller_integral_enable()
{
	pid_enable_integral(&roll_rate_controller);
	pid_enable_integral(&pitch_rate_controller);
	pid_enable_integral(&yaw_rate_controller);
}
void attitude_controller_integral_enable()
{
	pid_enable_integral(&roll_controller);
	pid_enable_integral(&pitch_controller);
}