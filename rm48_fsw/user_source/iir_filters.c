/*
 * File: iir_filters.c
 *
 * Author: Abhimanyu Ghosh
 * 			Controls and Robotics Research Laboratory (CRRL)
 * 			NYU Polytechnic School of Engineering
 * 			(c) 2014-2015
 */

#include "iir_filters.h"

void init_lpf_variables(lpf_data_struct* lpf_str, float cutoff_freq_hertz)
{
	lpf_str->prev_output = 0.0f;
	float rc_val = 1.0f/(2.0f*PI_VAL*cutoff_freq_hertz);
	lpf_str->alpha_val = DT_FILTER_LOOP/(DT_FILTER_LOOP+rc_val);
}

/*
 * Standard 1st-order Low-pass filter implementation, algorithmic inspiration from:
 * http://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
 */

float lowpass_filter(float input, lpf_data_struct* lpf_str)
{
	float output_value = (lpf_str->alpha_val*input) + ((1.0f-lpf_str->alpha_val)*lpf_str->prev_output);
	lpf_str->prev_output = output_value;
	return output_value;
}

void init_hpf_variables(hpf_data_struct* hpf_str, float cutoff_freq_hertz)
{
	hpf_str->prev_input = 0.0f;
	hpf_str->prev_output = 0.0f;
	float rc_val = 1.0f/(2.0f*PI_VAL*cutoff_freq_hertz);
	hpf_str->alpha_val = rc_val/(DT_FILTER_LOOP+rc_val);
}

/*
 * Standard 1st-order High-pass filter implementation, algorithmic inspiration from:
 * http://en.wikipedia.org/wiki/High-pass_filter#Discrete-time_realization
 */

float highpass_filter(float input, hpf_data_struct* hpf_str)
{
	float output_value = (hpf_str->alpha_val*hpf_str->prev_output)
									+ (hpf_str->alpha_val*(input-hpf_str->prev_input));
	hpf_str->prev_input = input;
	hpf_str->prev_output = output_value;
	return output_value;
}
