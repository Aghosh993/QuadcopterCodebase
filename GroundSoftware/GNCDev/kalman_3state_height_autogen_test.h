/******************************************************************************
 *                       Code generated with sympy 1.0                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *              This file is part of 'Kalman_3state_height_Core'              *
 ******************************************************************************/


#ifndef KALMAN_3STATE_HEIGHT_CORE__KALMAN_3STATE_HEIGHT_AUTOGEN_TEST__H
#define KALMAN_3STATE_HEIGHT_CORE__KALMAN_3STATE_HEIGHT_AUTOGEN_TEST__H

void predict_vertical_state(float a_dyn, float dt, float h, float v, float *out_492629177674309384);
void predict_vertical_cov(float dt, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float q11, float q22, float q33, float *out_5476348362250161562);
void update_vertical_gain(float p11, float p13, float p21, float p23, float p31, float p33, float r11, float r22, float *out_8020051339082852202);
void update_vertical_state(float a_dyn, float az, float d_lidar, float g, float g11, float g12, float g21, float g22, float g31, float g32, float h, float v, float *out_8699371518319484825);
void update_vertical_cov(float g11, float g12, float g21, float g22, float g31, float g32, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float *out_3791832019188488155);
void lidar_update_vertical_gain(float p11, float p21, float p31, float r_lidar, float *out_4466161268775116645);
void lidar_update_vertical_state(float a_dyn, float d_lidar, float g11, float g21, float g31, float h, float v, float *out_3494384365063633279);
void lidar_update_vertical_cov(float g11, float g21, float g31, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float *out_83866573131892437);
void accel_update_vertical_gain(float p13, float p23, float p33, float r_accel, float *out_8346643870775845793);
void accel_update_vertical_state(float a_dyn, float az, float g, float g11, float g21, float g31, float h, float v, float *out_4380166157680502389);
void accel_update_vertical_cov(float g11, float g21, float g31, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float *out_2345034652353781171);

#endif

