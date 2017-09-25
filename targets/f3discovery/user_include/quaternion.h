#ifndef QUATERNION_H_
#define QUATERNION_H_	1

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "vector3d.h"

typedef struct {
	float qx;
	float qy;
	float qz;
	float qw;
} quaternion_t;

quaternion_t quaternion_fromRotation(float angle, vector3d_t eulerComponents);
void quaternion_normalize(quaternion_t *q);
float quaternion_mag(quaternion_t q);
quaternion_t quaternion_add(quaternion_t a, quaternion_t b);
void quaternion_accumulate(quaternion_t *dst, quaternion_t src);
quaternion_t quaternion_mul(quaternion_t a, quaternion_t b);
quaternion_t quaternion_conjugate(quaternion_t q);
quaternion_t quaternion_inverse(quaternion_t q);
quaternion_t quaternion_rotate(quaternion_t q, quaternion_t rotationQuat);
vector3d_t quaternion_rotateVect3d(vector3d_t v, quaternion_t rotationQuat);
/*
	Rotate a vector v by inverse rotation specified by 
	valid unit rotation quaternion rotationQuat:
 */
vector3d_t quaternion_invRotateVect3d(vector3d_t v, quaternion_t rotationQuat);

vector3d_t quaternion_toEuler(quaternion_t q);
quaternion_t vector3d_toQuaternion(vector3d_t e);

#endif