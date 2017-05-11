#include "complementary_filter.h"

static float normalize_vector(float *data, int n)
{
	float mag = 0.0f;

	int i = 0;
	for(i=0; i<n; ++i)
	{
		mag += data[i] * data[i];
	}

	mag = sqrtf(mag);
	
	if(mag != 0.0f)
	{
		for(i=0; i<n; ++i)
		{
			data[i] /= mag;
		}
	}
}

static float scale_vector(float *data, float newmag, int n)
{
	float mag = 0.0f;

	int i = 0;
	for(i=0; i<n; ++i)
	{
		mag += data[i] * data[i];
	}

	mag = sqrtf(mag);

	float scale_factor = newmag/mag;
	
	if(mag != 0.0f)
	{
		for(i=0; i<n; ++i)
		{
			data[i] *= scale_factor;
		}
	}
}

static float roll_accel(complementary_filter_struct *s)
{
	float roll_sine = s->imu_data.accel_data[1]/(9.810f * cosf(s->state_vector.pitch));
	if(roll_sine > 1.0f)
	{
		roll_sine = 1.0f;
	}
	if(roll_sine < -1.0f)
	{
		roll_sine = -1.0f;
	}
	return asinf(roll_sine);
}

static float pitch_accel(complementary_filter_struct *s)
{
	return asinf(-1.0f*s->imu_data.accel_data[0]/(float)9.810f);
}

static void update_roll(complementary_filter_struct *s)
{
	float error;
	if(fabs(s->state_vector.pitch) < 80.0f)
	{
		error = roll_accel(s) - s->state_vector.roll;
	}
	else
	{
		error = 0.0f; // Just use the gyro through a singularity
	}

	s->roll_integral += error * s->filter_dt_seconds;

	float adj = error * s->k_P +  s->roll_integral * s->k_I;

	s->state_vector.roll_rate = degrees_to_radians(s->imu_data.gyro_data[0]) + adj;
	s->state_vector.roll += s->state_vector.roll_rate * s->filter_dt_seconds;
}

static void update_pitch(complementary_filter_struct *s)
{
	float error = pitch_accel(s) - s->state_vector.pitch;
	s->pitch_integral += error * s->filter_dt_seconds;

	float adj = error * s->k_P +  s->pitch_integral * s->k_I;

	s->state_vector.pitch_rate = degrees_to_radians(s->imu_data.gyro_data[1]) + adj;
	s->state_vector.pitch += s->state_vector.pitch_rate * s->filter_dt_seconds;
}

int init_complementary_filter(complementary_filter_struct *s, ACC_SCALE a, GYRO_SCALE g, MAG_SCALE m, 
								float dt_sec, float omega_natural, float damping_ratio)
{
	s->state_vector.roll = 0.0f;
	s->state_vector.pitch = 0.0f;
	s->state_vector.yaw = 0.0f;

	s->state_vector.roll_rate = 0.0f;
	s->state_vector.pitch_rate = 0.0f;
	s->state_vector.yaw_rate = 0.0f;

	s->state_vector.x = 0.0f;
	s->state_vector.y = 0.0f;
	s->state_vector.z = 0.0f;

	s->state_vector.v_x = 0.0f;
	s->state_vector.v_y = 0.0f;
	s->state_vector.v_z = 0.0f;

	s->state_vector.a_x = 0.0f;
	s->state_vector.a_y = 0.0f;
	s->state_vector.a_z = 0.0f;

	s->sensor_data.gps_lat = 0.0f;
	s->sensor_data.gps_long = 0.0f;
	s->sensor_data.gps_speed = 0.0f;
	s->sensor_data.gps_course = 0.0f;

	s->sensor_data.optical_flow_v_x = 0.0f;
	s->sensor_data.optical_flow_v_y = 0.0f;

	s->sensor_data.sonar_height = 0.0f;
	s->sensor_data.lidar_height = 0.0f;
	s->sensor_data.pressure_pa = 0.0f;

	s->filter_dt_seconds = dt_sec;

	s->k_P = 2.0f * omega_natural * damping_ratio;
	s->k_I = omega_natural * omega_natural;

	s->roll_integral = 0.0f;
	s->pitch_integral = 0.0f;

	return initialize_imu(a, g, m, &(s->imu_data));
}

void update_complementary_filter(complementary_filter_struct *s)
{
	get_scaled_imu_data(&(s->imu_data));
	scale_vector(&(s->imu_data.accel_data[0]), 9.810f, 3);
	update_pitch(s);
	update_roll(s);
}

float degrees_to_radians(float deg)
{
	return deg * (float)M_PI/180.0f;
}

float radians_to_degrees(float rad)
{
	return rad * 180.0f/(float)M_PI;
}

#if defined USE_STDIO_DEBUG
	void test_imu_acquisition(complementary_filter_struct *s)
	{
	    printf("Gyro: Roll %f Pitch %f Yaw %f\r\n", s->imu_data.gyro_data[0],
					                                  s->imu_data.gyro_data[1],
					                                  s->imu_data.gyro_data[2]);

	    printf("Accel: X %f Y %f Z %f\r\n", s->imu_data.accel_data[0],
	                                  		s->imu_data.accel_data[1],
	                                  		s->imu_data.accel_data[2]);

	    printf("Magnetometer: X %f Y %f Z %f\r\n", s->imu_data.magnetometer_data[0],
	                                           		s->imu_data.magnetometer_data[1],
	                                           		s->imu_data.magnetometer_data[2]);
	}
#endif