#include "vector3d.h"

vector3d_t addVect3d(vector3d_t v1, vector3d_t v2)
{
	vector3d_t ret;
	ret.i = v1.i + v2.i;
	ret.j = v1.j + v2.j;
	ret.k = v1.k + v2.k;
	return ret;
}

vector3d_t scaleVect3d(vector3d_t v, float scaleFactor)
{
	vector3d_t ret;
	ret.i = v.i * scaleFactor;
	ret.j = v.j * scaleFactor;
	ret.k = v.k * scaleFactor;
	return ret;
}

float dotVect3d(vector3d_t v1, vector3d_t v2)
{
	return v1.i*v2.i + v1.j*v2.j + v1.k*v2.k;
}

float magVect3d(vector3d_t v)
{
	return sqrtf(v.i*v.i + v.j*v.j + v.k*v.k);
}

vector3d_t crossProd(vector3d_t v1, vector3d_t v2)
{
	vector3d_t ret;
	ret.i = v1.j*v2.k - v1.k*v2.j;
	ret.j = v1.k*v2.i - v1.i*v2.k;
	ret.k = v1.i*v2.j - v1.j*v2.i;
	return ret;
}