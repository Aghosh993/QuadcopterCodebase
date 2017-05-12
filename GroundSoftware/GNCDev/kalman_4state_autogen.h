/******************************************************************************
 *                       Code generated with sympy 1.0                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                       This file is part of 'project'                       *
 ******************************************************************************/


#ifndef PROJECT__KALMAN_4STATE_AUTOGEN__H
#define PROJECT__KALMAN_4STATE_AUTOGEN__H

void predict_state(double dt, double pitch, double pitch_rate, double roll, double roll_rate, double *out_4934366627997458542);
void predict_cov(double dt, double p11, double p12, double p13, double p14, double p21, double p22, double p23, double p24, double p31, double p32, double p33, double p34, double p41, double p42, double p43, double p44, double q11, double q22, double q33, double q44, double *out_7046910104081708530);
void update_gain(double g, double p11, double p12, double p13, double p14, double p21, double p22, double p23, double p24, double p31, double p32, double p33, double p34, double p41, double p42, double p43, double p44, double r11, double r22, double r33, double r44, double *out_2564730263248370446);
void update_state(double ax, double ay, double g, double g11, double g12, double g13, double g14, double g21, double g22, double g23, double g24, double g31, double g32, double g33, double g34, double g41, double g42, double g43, double g44, double gyro_pitch, double gyro_roll, double pitch, double pitch_rate, double roll, double roll_rate, double *out_1048260822836027695);
void update_cov(double g, double g11, double g12, double g13, double g14, double g21, double g22, double g23, double g24, double g31, double g32, double g33, double g34, double g41, double g42, double g43, double g44, double p11, double p12, double p13, double p14, double p21, double p22, double p23, double p24, double p31, double p32, double p33, double p34, double p41, double p42, double p43, double p44, double *out_1617170561140791062);

#endif

