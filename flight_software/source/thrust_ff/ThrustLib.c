/*
 * ThrustLib.c
 *
 *  Created on: Feb 1, 2014
 *      Author: aghosh01
 */

// Library header:
#include "ThrustLib.h"

float thrust_table[LOOKUP_TABLE_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
					 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
					28, 29, 30, 31, 32, 33, 34, 35, 36, 37};

float pwm_table[LOOKUP_TABLE_SIZE] = {0.049, 0.05, 0.051, 0.052, 0.053, 0.054, 0.055, 0.056,
				      0.057, 0.058, 0.059, 0.06, 0.061, 0.062, 0.063, 0.064,
			  	      0.065, 0.066, 0.067, 0.068, 0.069, 0.07, 0.071, 0.072,
				      0.073, 0.074, 0.075, 0.076, 0.077, 0.078, 0.079, 0.08,
				      0.081, 0.082, 0.083, 0.084, 0.085, 0.086};

float thrust_to_pwm_openloop(float thrust_requested)
{
	int i = 0;
	float m_inverse_secant_local = 0.0f;

	for(i=0;i<LOOKUP_TABLE_SIZE-1;++i)
	{
		if(thrust_requested >= thrust_table[i] && thrust_requested < thrust_table[i+1])
		{
			m_inverse_secant_local = (pwm_table[i+1]-pwm_table[i])/(thrust_table[i+1]-thrust_table[i]);
			return pwm_table[i] + ( (thrust_requested-thrust_table[i]) * m_inverse_secant_local );
		}
	}
	return -1;
}
