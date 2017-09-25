#ifndef NAV_COMP_FILTER_H_
#define NAV_COMP_FILTER_H_	1

#include "vector3d.h"
#include "quaternion.h"
#include "imu_hal.h"

#define ENULLIMUDESC	-1
#define ENOACCEL		-2
#define ENOGYRO			-3

typedef struct {
	imu_desc *sourceImu;

	quaternion_t attitudeCurrent;
	quaternion_t attitudeRateCurrent;

	float _kP;
	float _kI;
	float _dt;
} comp_filter_t;

int compfilterInit(comp_filter_t *c, imu_desc *imuResource, 
					float filter_kp, float filter_ki, float filter_dt);

#endif