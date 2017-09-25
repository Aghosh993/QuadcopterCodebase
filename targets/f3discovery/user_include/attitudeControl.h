#ifndef ATTITUDE_CONTROL_H_	
#define ATTITUDE_CONTROL_H_		1

#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#include "vector3d.h"
#include "quaternion.h"

typedef struct {
	float attitude_gain;
	float rate_gain;
} attitudeController_t;

void attitudeControllerInit(attitudeController_t *c, 
								float attitudeGain, float rateDamperGain);
vector3d_t attitudeController_computeTorques(attitudeController_t c, 
												quaternion_t setpoint, 
												quaternion_t actual_attitude, 
												vector3d_t bodyRatesActual);

#endif