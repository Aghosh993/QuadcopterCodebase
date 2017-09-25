#ifndef VECTOR3D_H_
#define VECTOR3D_H_		1

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	float i;
	float j;
	float k;
} vector3d_t;

vector3d_t addVect3d(vector3d_t v1, vector3d_t v2);
vector3d_t scaleVect3d(vector3d_t v, float scaleFactor);
float dotVect3d(vector3d_t v1, vector3d_t v2);
float magVect3d(vector3d_t v);
vector3d_t crossProd(vector3d_t v1, vector3d_t v2);

#endif