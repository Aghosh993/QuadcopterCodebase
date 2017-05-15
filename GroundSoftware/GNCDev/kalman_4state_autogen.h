/******************************************************************************
 *                       Code generated with sympy 1.0                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                 This file is part of 'Kalman_4state_Core'                  *
 ******************************************************************************/


#ifndef KALMAN_4STATE_CORE__KALMAN_4STATE_AUTOGEN__H
#define KALMAN_4STATE_CORE__KALMAN_4STATE_AUTOGEN__H

void predict_state(float dt, float pitch, float pitch_rate, float roll, float roll_rate, float *out_6256484655591047007);
void predict_cov(float dt, float p11, float p12, float p13, float p14, float p21, float p22, float p23, float p24, float p31, float p32, float p33, float p34, float p41, float p42, float p43, float p44, float q11, float q22, float q33, float q44, float *out_874294713084759149);
void update_gain(float g, float p11, float p12, float p13, float p14, float p21, float p22, float p23, float p24, float p31, float p32, float p33, float p34, float p41, float p42, float p43, float p44, float r11, float r22, float r33, float r44, float *out_7497920569122251633);
void update_state(float ax, float ay, float g, float g11, float g12, float g13, float g14, float g21, float g22, float g23, float g24, float g31, float g32, float g33, float g34, float g41, float g42, float g43, float g44, float gyro_pitch, float gyro_roll, float pitch, float pitch_rate, float roll, float roll_rate, float *out_309013171901928691);
void update_cov(float g, float g11, float g12, float g13, float g14, float g21, float g22, float g23, float g24, float g31, float g32, float g33, float g34, float g41, float g42, float g43, float g44, float p11, float p12, float p13, float p14, float p21, float p22, float p23, float p24, float p31, float p32, float p33, float p34, float p41, float p42, float p43, float p44, float *out_4012366199454581357);

#endif

