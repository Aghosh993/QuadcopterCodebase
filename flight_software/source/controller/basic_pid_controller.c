/*
 * @File: 	basic_pid_controller.c
 * @Author: Abhimanyu Ghosh
 * @Date: 	10/02/2015
  
 * @Brief: 			General-purpose PID (Proportional-Integral-Derivative) controller library.
 * @Description: 	Implements the basic functions required for the initialization, control and update of generic
 					    PID controllers
 */

#include "basic_pid_controller.h"

void init_pid_controller(pid_data_struct* str, float k_p, float k_i, float k_d,
							float dt, float integral_windup_max_abs_val,
							float pid_max_adjustment_abs_val)
{
	str->accumulated_error = 0.0f;
	str->k_p = k_p;
	str->k_i = k_i;
	str->k_d = k_d;
	str->dt = dt;
	str->control_enable_state = CONTROLLER_DISABLED;
	
	#ifndef DERIVATIVE_ON_MEASUREMENT
		str->last_error_value = 0.0f;
	#endif

	#ifdef DERIVATIVE_ON_MEASUREMENT
		str->last_state_measurement = 0.0f;
	#endif
	
	str->error_saturation_absolute_value = integral_windup_max_abs_val;
	str->maximum_pid_adjustment_value = pid_max_adjustment_abs_val;

	str->pid_struct_checksum = calc_pid_struct_checksum(*str);
}

void init_pi_d_lpf_controller(pi_d_lpf_data_struct* str, float k_p, float k_i, float k_d,
									float dt, float integral_windup_max_abs_val,
									float pid_max_adjustment_abs_val,
									float derivative_lpf_cutoff_freq_hertz)
{
	str->accumulated_error = 0.0f;
	str->k_p = k_p;
	str->k_i = k_i;
	str->k_d = k_d;
	str->dt = dt;
	str->control_enable_state = CONTROLLER_DISABLED;
	
	#ifndef DERIVATIVE_ON_MEASUREMENT
		str->last_error_value = 0.0f;
	#endif

	#ifdef DERIVATIVE_ON_MEASUREMENT
		str->last_state_measurement = 0.0f;
	#endif

	str->error_saturation_absolute_value = integral_windup_max_abs_val;
	str->maximum_pid_adjustment_value = pid_max_adjustment_abs_val;
	init_lpf_variables(&(str->derivative_lpf_struct), derivative_lpf_cutoff_freq_hertz);

	str->pid_struct_checksum = calc_pi_d_lpf_struct_checksum(*str);
}

void enable_pid_controller(pid_data_struct* str)
{
	str->control_enable_state = CONTROLLER_ENABLED;
	str->pid_struct_checksum = calc_pid_struct_checksum(*str);
}

void enable_pi_d_lpf_controller(pi_d_lpf_data_struct* str)
{
	str->control_enable_state = CONTROLLER_ENABLED;
	str->pid_struct_checksum = calc_pi_d_lpf_struct_checksum(*str);
}

void disable_pid_controller(pid_data_struct* str)
{
	str->control_enable_state = CONTROLLER_DISABLED;
	str->pid_struct_checksum = calc_pid_struct_checksum(*str);
}

void disable_pi_d_lpf_controller(pi_d_lpf_data_struct* str)
{
	str->control_enable_state = CONTROLLER_DISABLED;
	str->pid_struct_checksum = calc_pi_d_lpf_struct_checksum(*str);
}

#ifndef DERIVATIVE_ON_MEASUREMENT

float calculate_pid_adjustment(pid_data_struct* str, float state_measurement, float commanded_state)
{
	float error = commanded_state - state_measurement;

	// Compute proportional term:
	float proportional_adjustment = error*str->k_p;

	// Compute derivative term:
	float derivative_adjustment = (error - str->last_error_value)*str->k_d/str->dt;

	// Accumulate the integral and check for wind-up, add saturation:
	str->accumulated_error += error*str->dt;

	if(str->accumulated_error >= 0.0f)
	{
		if(str->accumulated_error > str->error_saturation_absolute_value)
		{
			str->accumulated_error = str->error_saturation_absolute_value;
		}
	}
	else
	{
		if(str->accumulated_error < -1.0f * str->error_saturation_absolute_value)
		{
			str->accumulated_error = -1.0f * str->error_saturation_absolute_value;
		}
	}

	// Compute integral term from accumulated error in roll rate:
	float integral_adjustment = str->accumulated_error*str->k_i;

	// Calculate net PID adjustment:
	float pid_net_adjustment = proportional_adjustment +
							derivative_adjustment +
							integral_adjustment;

	// Limit the maximum PID adjustment:
	if(pid_net_adjustment >= 0.0f)
	{
		if(pid_net_adjustment > str->maximum_pid_adjustment_value)
		{
			pid_net_adjustment = str->maximum_pid_adjustment_value;
		}
	}
	else
	{
		if(pid_net_adjustment < -1.0f * str->maximum_pid_adjustment_value)
		{
			pid_net_adjustment = -1.0f * str->maximum_pid_adjustment_value;
		}
	}

	// Update the previous roll and roll rate values for next loop iteration:
	str->last_error_value = error;
	// Compute and update PID struct checksum;
	str->pid_struct_checksum = calc_pid_struct_checksum(*str);
	return pid_net_adjustment;
}

float calculate_pi_d_lpf_adjustment(pi_d_lpf_data_struct* str, float state_measurement, float commanded_state)
{
	float error = commanded_state - state_measurement;

	// Compute proportional term:
	float proportional_adjustment = error*str->k_p;

	// Compute derivative term:
	float raw_derivative = (error - str->last_error_value)/str->dt;
	float filtered_derivative = lowpass_filter(raw_derivative, &(str->derivative_lpf_struct));
	float derivative_adjustment = filtered_derivative * str->k_d;

	// Accumulate the integral and check for wind-up, add saturation:
	str->accumulated_error += error*str->dt;

	if(str->accumulated_error >= 0.0f)
	{
		if(str->accumulated_error > str->error_saturation_absolute_value)
		{
			str->accumulated_error = str->error_saturation_absolute_value;
		}
	}
	else
	{
		if(str->accumulated_error < -1.0f * str->error_saturation_absolute_value)
		{
			str->accumulated_error = -1.0f * str->error_saturation_absolute_value;
		}
	}

	// Compute integral term from accumulated error in roll rate:
	float integral_adjustment = str->accumulated_error*str->k_i;

	// Calculate net PID adjustment:
	float pid_net_adjustment = proportional_adjustment +
							derivative_adjustment +
							integral_adjustment;

	// Limit the maximum PID adjustment:
	if(pid_net_adjustment >= 0.0f)
	{
		if(pid_net_adjustment > str->maximum_pid_adjustment_value)
		{
			pid_net_adjustment = str->maximum_pid_adjustment_value;
		}
	}
	else
	{
		if(pid_net_adjustment < -1.0f * str->maximum_pid_adjustment_value)
		{
			pid_net_adjustment = -1.0f * str->maximum_pid_adjustment_value;
		}
	}

	// Update the previous roll and roll rate values for next loop iteration:
	str->last_error_value = error;
	// Compute and update PID struct checksum;
	str->pid_struct_checksum = calc_pi_d_lpf_struct_checksum(*str);
	return pid_net_adjustment;
}

#endif

/*
	Based on http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/

	Basically, computing the derivative term purely based on the rate of change of the state variable measured
	leads to improved immunity to step changes in the setpoint.
 */

#ifdef DERIVATIVE_ON_MEASUREMENT

float calculate_pid_adjustment(pid_data_struct* str, float state_measurement, float commanded_state)
{
	float error = commanded_state - state_measurement;

	// Compute proportional term:
	float proportional_adjustment = error*str->k_p;

	// Compute derivative term:
	float derivative_adjustment = (state_measurement - str->last_state_measurement)*-1.0f*str->k_d/str->dt;

	// Accumulate the integral and check for wind-up, add saturation:
	str->accumulated_error += error*str->dt;

	if(str->accumulated_error >= 0.0f)
	{
		if(str->accumulated_error > str->error_saturation_absolute_value)
		{
			str->accumulated_error = str->error_saturation_absolute_value;
		}
	}
	else
	{
		if(str->accumulated_error < -1.0f * str->error_saturation_absolute_value)
		{
			str->accumulated_error = -1.0f * str->error_saturation_absolute_value;
		}
	}

	// Compute integral term from accumulated error in roll rate:
	float integral_adjustment = str->accumulated_error*str->k_i;

	// Calculate net PID adjustment:
	float pid_net_adjustment = proportional_adjustment -
							derivative_adjustment +
							integral_adjustment;

	// Limit the maximum PID adjustment:
	if(pid_net_adjustment >= 0.0f)
	{
		if(pid_net_adjustment > str->maximum_pid_adjustment_value)
		{
			pid_net_adjustment = str->maximum_pid_adjustment_value;
		}
	}
	else
	{
		if(pid_net_adjustment < -1.0f * str->maximum_pid_adjustment_value)
		{
			pid_net_adjustment = -1.0f * str->maximum_pid_adjustment_value;
		}
	}

	// Update the previous roll and roll rate values for next loop iteration:
	str->last_state_measurement = state_measurement;
	// Compute and update PID struct checksum;
	str->pid_struct_checksum = calc_pid_struct_checksum(*str);
	return pid_net_adjustment;
}

float calculate_pi_d_lpf_adjustment(pi_d_lpf_data_struct* str, float state_measurement, float commanded_state)
{
	float error = commanded_state - state_measurement;

	// Compute proportional term:
	float proportional_adjustment = error*str->k_p;

	// Compute derivative term:
	float raw_derivative = (state_measurement - str->last_state_measurement)/str->dt;
	float filtered_derivative = lowpass_filter(raw_derivative, &(str->derivative_lpf_struct));
	float derivative_adjustment = filtered_derivative * -1.0f * str->k_d;

	// Accumulate the integral and check for wind-up, add saturation:
	str->accumulated_error += error*str->dt;

	if(str->accumulated_error >= 0.0f)
	{
		if(str->accumulated_error > str->error_saturation_absolute_value)
		{
			str->accumulated_error = str->error_saturation_absolute_value;
		}
	}
	else
	{
		if(str->accumulated_error < -1.0f * str->error_saturation_absolute_value)
		{
			str->accumulated_error = -1.0f * str->error_saturation_absolute_value;
		}
	}

	// Compute integral term from accumulated error in roll rate:
	float integral_adjustment = str->accumulated_error*str->k_i;

	// Calculate net PID adjustment:
	float pid_net_adjustment = proportional_adjustment -
							derivative_adjustment +
							integral_adjustment;

	// Limit the maximum PID adjustment:
	if(pid_net_adjustment >= 0.0f)
	{
		if(pid_net_adjustment > str->maximum_pid_adjustment_value)
		{
			pid_net_adjustment = str->maximum_pid_adjustment_value;
		}
	}
	else
	{
		if(pid_net_adjustment < -1.0f * str->maximum_pid_adjustment_value)
		{
			pid_net_adjustment = -1.0f * str->maximum_pid_adjustment_value;
		}
	}

	// Update the previous roll and roll rate values for next loop iteration:
	str->last_state_measurement = state_measurement;
	// Compute and update PID struct checksum;
	str->pid_struct_checksum = calc_pi_d_lpf_struct_checksum(*str);
	return pid_net_adjustment;
}

#endif

// Some auxiliary functions:

void copy_pid_struct_data(pid_data_struct* dest, const pid_data_struct src)
{
	dest->accumulated_error = src.accumulated_error;
	dest->maximum_pid_adjustment_value = src.maximum_pid_adjustment_value;
	dest->dt = src.dt;
	dest->control_enable_state = src.control_enable_state;
	dest->error_saturation_absolute_value = src.error_saturation_absolute_value;

	#ifndef DERIVATIVE_ON_MEASUREMENT
		dest->last_error_value = src.last_error_value;
	#endif
	#ifdef DERIVATIVE_ON_MEASUREMENT
		dest->last_state_measurement = src.last_state_measurement;
	#endif

	dest->pid_struct_checksum = src.pid_struct_checksum;
}

void copy_pi_d_lpf_struct_data(pi_d_lpf_data_struct* dest, const pi_d_lpf_data_struct src)
{
	dest->accumulated_error = src.accumulated_error;
	dest->maximum_pid_adjustment_value = src.maximum_pid_adjustment_value;
	dest->dt = src.dt;
	dest->control_enable_state = src.control_enable_state;
	dest->error_saturation_absolute_value = src.error_saturation_absolute_value;

	#ifndef DERIVATIVE_ON_MEASUREMENT
		dest->last_error_value = src.last_error_value;
	#endif
	#ifdef DERIVATIVE_ON_MEASUREMENT
		dest->last_state_measurement = src.last_state_measurement;
	#endif
		
	dest->pid_struct_checksum = src.pid_struct_checksum;

	dest->derivative_lpf_struct.prev_output = src.derivative_lpf_struct.prev_output;
	dest->derivative_lpf_struct.alpha_val = src.derivative_lpf_struct.alpha_val;	
}

/*
 * A simple checksumming algorithm for PID data structures utilizing the LRC (Longitudinal
 * Redundancy Check) Algorithm as described in:
 * http://en.wikipedia.org/wiki/Longitudinal_redundancy_check
 */

uint8_t calc_pid_struct_checksum(pid_data_struct str)
{
	union {
		pid_data_struct st;
		uint8_t st_data[PID_STRUCT_SIZE_BYTES];
	} pid_struct_to_bytes;

	copy_pid_struct_data(&(pid_struct_to_bytes.st), str);

	uint8_t i = 0U;
	uint8_t running_lrc_sum = 0U;

	for(i=0U; i<PID_STRUCT_SIZE_BYTES-1; ++i) // Include all but the last checksum byte in the struct in our LRC computation
	{
		running_lrc_sum = (running_lrc_sum + pid_struct_to_bytes.st_data[i]) & 0xFFU;
	}
	running_lrc_sum ^= 0xFFU;
	running_lrc_sum += 1;
	running_lrc_sum &= 0xFFU;
	return running_lrc_sum;
}

uint8_t calc_pi_d_lpf_struct_checksum(pi_d_lpf_data_struct str)
{
	union {
		pi_d_lpf_data_struct st;
		uint8_t st_data[PI_D_LPF_STRUCT_SIZE_BYTES];
	} pid_struct_to_bytes;

	copy_pi_d_lpf_struct_data(&(pid_struct_to_bytes.st), str);

	uint8_t i = 0U;
	uint8_t running_lrc_sum = 0U;

	for(i=0U; i<PI_D_LPF_STRUCT_SIZE_BYTES-1; ++i) // Include all but the last checksum byte in the struct in our LRC computation
	{
		running_lrc_sum = (running_lrc_sum + pid_struct_to_bytes.st_data[i]) & 0xFFU;
	}
	running_lrc_sum ^= 0xFFU;
	running_lrc_sum += 1;
	running_lrc_sum &= 0xFFU;
	return running_lrc_sum;
}

pid_checksum_result verify_pid_struct_checksum(const pid_data_struct str)
{
	uint8_t calculated_checksum = calc_pid_struct_checksum(str);
	if(calculated_checksum == str.pid_struct_checksum)
	{
		return PID_CHECKSUM_PASS;
	}
	return PID_CHECKSUM_FAIL;
}

pid_checksum_result verify_pi_d_lpf_struct_checksum(const pi_d_lpf_data_struct str)
{
	uint8_t calculated_checksum = calc_pi_d_lpf_struct_checksum(str);
	if(calculated_checksum == str.pid_struct_checksum)
	{
		return PID_CHECKSUM_PASS;
	}
	return PID_CHECKSUM_FAIL;
}

int init_pid_feedforward_data_struct(pid_ff_data_struct* str, uint32_t n_data_points, 
										float* error_array_input, float* actuation_commands_array_input)
{
	// if()
}