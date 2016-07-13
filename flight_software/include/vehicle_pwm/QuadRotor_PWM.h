/*
 * QuadRotor_PWM.h
 *
 *  Created on: Jul 13, 2014
 *      Author: aghosh01
 */

#ifndef QUADROTOR_PWM_H_
#define QUADROTOR_PWM_H_

#include "QuadRotor_PWM_hal.h"

/*
 * Set all motors inidivdually to an array of throttle values
 * @Argument: A double-precision array of thrust values that can range from 0.0 to 1.0,
 * 				with 0.0 indicating a complete stop (1 ms pulse), and 1.0 indicating
 * 				full throttle (i.e. 2 ms RC pulse on the PWM channel).
 * 				The order is in order of motor number (i.e. array index i indicates
 * 				motor # i+1 (Ex: thrust[0] holds the throttle value for motor 1 on the vehicle)
 */
void QuadRotor_set_all_motors(double* thrust);

#endif /* QUADROTOR_PWM_H_ */
