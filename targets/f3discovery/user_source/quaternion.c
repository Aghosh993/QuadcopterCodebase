/*
	A simple quaternion API that manipulates vector/quaternion-like objects
	Portions of this code developed with reference made to:

	1) https://github.com/daPhoosa/SimpleIMU-6/blob/master/SimpleIMU-6.ino
	2) https://en.wikipedia.org/wiki/Dot_product
	3) http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf
	4) https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons
	5) http://stanford.edu/class/ee267/lectures/lecture10.pdf

	(c) 2017, Abhimanyu Ghosh
 */

#include "quaternion.h"

quaternion_t quaternion_fromRotation(float angle, vector3d_t eulerComponents)
{
	quaternion_t qret;

	float sinHalfTheta = sinf(angle/2.0f);
	float cosineHalfTheta = cosf(angle/2.0f);

	qret.qx = eulerComponents.i * sinHalfTheta;
	qret.qy = eulerComponents.j * sinHalfTheta;
	qret.qz = eulerComponents.k * sinHalfTheta;
	qret.qw = cosineHalfTheta;

	return qret;
}

void quaternion_normalize(quaternion_t *q)
{
	float qmag = quaternion_mag(*q);
	if(qmag < -0.05f || qmag > 0.05f)
	{
		q->qx /= qmag;
		q->qy /= qmag;
		q->qz /= qmag;
		q->qw /= qmag;
	}
}

float quaternion_mag(quaternion_t q)
{
	return sqrtf(q.qx*q.qx + q.qy*q.qy + q.qz*q.qz + q.qw*q.qw);
}

quaternion_t quaternion_add(quaternion_t a, quaternion_t b)
{
	quaternion_t qret;
	
	qret.qx = a.qx + b.qx;
	qret.qy = a.qy + b.qy;
	qret.qz = a.qz + b.qz;
	qret.qw = a.qw + b.qw;

	return qret;
}

void quaternion_accumulate(quaternion_t *dst, quaternion_t src)
{
	dst->qx += src.qx;
	dst->qy += src.qy;
	dst->qz += src.qz;
	dst->qw += src.qw;
}

quaternion_t quaternion_mul(quaternion_t a, quaternion_t b)
{
	quaternion_t qret;

	qret.qx = a.qw*b.qx + a.qx*b.qw + a.qy*b.qz - a.qz*b.qy;
	qret.qy = a.qw*b.qy - a.qx*b.qz + a.qy*b.qw + a.qz*b.qx;
	qret.qz = a.qw*b.qz + a.qx*b.qy - a.qy*b.qx + a.qz*b.qw;
	qret.qw = a.qw*b.qw - a.qx*b.qx - a.qy*b.qy - a.qz*b.qz;

	return qret;
}

quaternion_t quaternion_conjugate(quaternion_t q)
{
	quaternion_t qret;
	
	qret.qx = -1.0f*q.qx;
	qret.qy = -1.0f*q.qy;
	qret.qz = -1.0f*q.qz;
	qret.qw = q.qw;

	return qret;
}

quaternion_t quaternion_inverse(quaternion_t q)
{
	quaternion_t qret = quaternion_conjugate(q);
	float qmagSq = q.qx*q.qx + q.qy*q.qy + q.qz*q.qz + q.qw*q.qw;
	if(qmagSq > 1.0f)
	{
		qret.qx /= qmagSq;
		qret.qy /= qmagSq;
		qret.qz /= qmagSq;
		qret.qw /= qmagSq;
	}

	return qret;
}

/*
	Rotate quaternion q by angle defined by unit quaternion rotationQuat
	rotationQuat must be normalized and a valid rotation quaternion!!
 */
quaternion_t quaternion_rotate(quaternion_t q, quaternion_t rotationQuat)
{
	/*
		RotatedQuaternion = rotationQuaternion * q * rotationQuaternion^-1:
	 */
	return quaternion_mul(quaternion_mul(rotationQuat, q), quaternion_inverse(rotationQuat));
}

/*
	Rotate a vector v by rotation specified by 
	valid unit rotation quaternion rotationQuat:
 */
vector3d_t quaternion_rotateVect3d(vector3d_t v, quaternion_t rotationQuat)
{
	/*
		v_rotated = v + 2r x (r x v + wv)
		where v is the vector to be rotated, r is the imaginary part of rotationQuat,
		w is the real part of rotationQuat
		Note that rotationQuat must be a valid, unit-normalized rotation quaternion!! 
	 */
	vector3d_t r;
	r.i = rotationQuat.qx;
	r.j = rotationQuat.qy;
	r.k = rotationQuat.qz;

	vector3d_t vRotated;
	vRotated = addVect3d(v, 
						crossProd(scaleVect3d(r, 2.0f), 
								addVect3d(crossProd(r, v), scaleVect3d(v, rotationQuat.qw))));
	return vRotated;
}

/*
	Rotate a vector v by inverse rotation specified by 
	valid unit rotation quaternion rotationQuat:
 */
vector3d_t quaternion_invRotateVect3d(vector3d_t v, quaternion_t rotationQuat)
{
	/*
		v_rotated = v + 2r x (r x v + wv)
		where v is the vector to be rotated, r is the imaginary part of rotationQuat,
		w is the real part of rotationQuat
		Note that rotationQuat must be a valid, unit-normalized rotation quaternion!! 
	 */
	vector3d_t r;
	r.i = -1.0f*rotationQuat.qx;
	r.j = -1.0f*rotationQuat.qy;
	r.k = -1.0f*rotationQuat.qz;

	vector3d_t vRotated;
	vRotated = addVect3d(v, 
						crossProd(scaleVect3d(r, 2.0f), 
								addVect3d(crossProd(r, v), scaleVect3d(v, rotationQuat.qw))));
	return vRotated;
}

vector3d_t quaternion_toEuler(quaternion_t q)
{
	vector3d_t ret;

	quaternion_t input;
	input.qx = q.qx;
	input.qy = q.qy;
	input.qz = q.qz;
	input.qw = q.qw;

	quaternion_normalize(&input);

	ret.i = atan2f(2.0f*(input.qw*input.qx + input.qy*input.qz), (1.0f-(2.0f*(input.qx*input.qx + input.qy*input.qy))));
	ret.j = asinf(2.0f*(input.qw*input.qy - input.qz*input.qx));
	ret.k = atan2f(2.0f*(input.qw*input.qz + input.qx*input.qy), 1.0f-2.0f*(input.qy*input.qy + input.qz*input.qz));

	return ret;
}

/*
	Converts euler angles (in radians) provided in vector e into
	Quaternion format.

	Based on Euler-Quaternion conversions noted at:
	https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
quaternion_t vector3d_toQuaternion(vector3d_t e)
{
	quaternion_t qret;

	float rollCosine = 	cosf(e.i * 0.5f);
	float rollSine = 	sinf(e.i * 0.5f);
	float pitchCosine = cosf(e.j * 0.5f);
	float pitchSine = 	sinf(e.j * 0.5f);
	float yawCosine = 	cosf(e.k * 0.5f);
	float yawSine = 	sinf(e.k * 0.5f);

	qret.qw = rollCosine*pitchCosine*yawCosine + rollSine*pitchSine*yawSine;
	qret.qx = rollSine*pitchCosine*yawCosine   - rollCosine*pitchSine*yawSine;
	qret.qy = rollCosine*pitchSine*yawCosine   + rollSine*pitchCosine*yawSine;
	qret.qz = rollCosine*pitchCosine*yawSine   - rollSine*pitchSine*yawCosine;

	return qret;
}