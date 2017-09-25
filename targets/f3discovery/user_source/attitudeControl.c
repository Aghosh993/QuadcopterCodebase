#include "attitudeControl.h"

/*
	A quaternion-based attitude control library, based on:
	http://folk.ntnu.no/skoge/prost/proceedings/ecc-2013/data/papers/0927.pdf
	"Full Quaternion Based Attitude Control for a Quadrotor"
	(Emil Fresk, George Nikolakopoulos)
	Department of Computer, Electrical and Space Engineering
	Lulea University of Technology
	July 17-19, 2013, Zürich, Switzerland
	Published: 2013 European Control Conference (ECC)
 */

static void constrainAdj(vector3d_t *vect, float absLimit)
{
	if(vect->i > fabs(absLimit))
	{
		vect->i = fabs(absLimit);
	}

	if(vect->j > fabs(absLimit))
	{
		vect->j = fabs(absLimit);
	}

	if(vect->k > fabs(absLimit))
	{
		vect->k = fabs(absLimit);
	}

	if(vect->i < -1.0f*fabs(absLimit))
	{
		vect->i = -1.0f*fabs(absLimit);
	}

	if(vect->j < -1.0f*fabs(absLimit))
	{
		vect->j = -1.0f*fabs(absLimit);
	}

	if(vect->k < -1.0f*fabs(absLimit))
	{
		vect->k = -1.0f*fabs(absLimit);
	}
}

void attitudeControllerInit(attitudeController_t *c, 
								float attitudeGain, float rateDamperGain)
{
	c->attitude_gain = attitudeGain;
	c->rate_gain = rateDamperGain;
}

vector3d_t attitudeController_computeTorques(attitudeController_t c, 
												quaternion_t setpoint, 
												quaternion_t actual_attitude, 
												vector3d_t bodyRatesActual)
{
	vector3d_t adj = {0.0f, 0.0f, 0.0f};
	
	/*
		Determine error signal between desired and true attitude quaternions:
	 */
	quaternion_t qErr = quaternion_mul(setpoint, quaternion_conjugate(actual_attitude));

	vector3d_t errorAxisComponents;
	errorAxisComponents.i = qErr.qx;
	errorAxisComponents.j = qErr.qy;
	errorAxisComponents.k = qErr.qz;

	errorAxisComponents = scaleVect3d(errorAxisComponents, -1.0f*c.attitude_gain);
	adj = addVect3d(scaleVect3d(bodyRatesActual, -1.0f*c.rate_gain), errorAxisComponents);
	constrainAdj(&adj, 1.0f);
	
	return adj;
}