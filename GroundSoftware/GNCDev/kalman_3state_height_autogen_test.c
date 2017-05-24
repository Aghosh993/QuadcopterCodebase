/******************************************************************************
 *                       Code generated with sympy 1.0                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *              This file is part of 'Kalman_3state_height_Core'              *
 ******************************************************************************/
#include "kalman_3state_height_autogen_test.h"
#include <math.h>

void predict_vertical_state(float a_dyn, float dt, float h, float v, float *out_492629177674309384) {

   out_492629177674309384[0] = 0.5*a_dyn*dt*dt + dt*v + h;
   out_492629177674309384[1] = a_dyn*dt + v;
   out_492629177674309384[2] = a_dyn;

}

void predict_vertical_cov(float dt, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float q11, float q22, float q33, float *out_5476348362250161562) {

   out_5476348362250161562[0] = 0.5*dt*dt*p31 + 0.5*dt*dt*(0.5*dt*dt*p33 + dt*p23 + p13) + dt*p21 + dt*(0.5*dt*dt*p32 + dt*p22 + p12) + p11 + q11;
   out_5476348362250161562[1] = 0.5*dt*dt*p32 + dt*p22 + dt*(0.5*dt*dt*p33 + dt*p23 + p13) + p12;
   out_5476348362250161562[2] = 0.5*dt*dt*p33 + dt*p23 + p13;
   out_5476348362250161562[3] = 0.5*dt*dt*(dt*p33 + p23) + dt*p31 + dt*(dt*p32 + p22) + p21;
   out_5476348362250161562[4] = dt*p32 + dt*(dt*p33 + p23) + p22 + q22;
   out_5476348362250161562[5] = dt*p33 + p23;
   out_5476348362250161562[6] = 0.5*dt*dt*p33 + dt*p32 + p31;
   out_5476348362250161562[7] = dt*p33 + p32;
   out_5476348362250161562[8] = p33 + q33;

}

void update_vertical_gain(float p11, float p13, float p21, float p23, float p31, float p33, float r11, float r22, float *out_8020051339082852202) {

   out_8020051339082852202[0] = p11*(p33 + r22)/(-p13*p31 + (-p11 - r11)*(-p33 - r22)) - p13*p31/(-p13*p31 + (-p11 - r11)*(-p33 - r22));
   out_8020051339082852202[1] = -p11*p13/(-p13*p31 + (-p11 - r11)*(-p33 - r22)) + p13*(p11 + r11)/(-p13*p31 + (-p11 - r11)*(-p33 - r22));
   out_8020051339082852202[2] = p21*(p33 + r22)/(-p13*p31 + (-p11 - r11)*(-p33 - r22)) - p23*p31/(-p13*p31 + (-p11 - r11)*(-p33 - r22));
   out_8020051339082852202[3] = -p13*p21/(-p13*p31 + (-p11 - r11)*(-p33 - r22)) + p23*(p11 + r11)/(-p13*p31 + (-p11 - r11)*(-p33 - r22));
   out_8020051339082852202[4] = -p31*p33/(-p13*p31 + (-p11 - r11)*(-p33 - r22)) + p31*(p33 + r22)/(-p13*p31 + (-p11 - r11)*(-p33 - r22));
   out_8020051339082852202[5] = -p13*p31/(-p13*p31 + (-p11 - r11)*(-p33 - r22)) + p33*(p11 + r11)/(-p13*p31 + (-p11 - r11)*(-p33 - r22));

}

void update_vertical_state(float a_dyn, float az, float d_lidar, float g, float g11, float g12, float g21, float g22, float g31, float g32, float h, float v, float *out_8699371518319484825) {

   out_8699371518319484825[0] = g11*(d_lidar - h) + g12*(-a_dyn + az - g) + h;
   out_8699371518319484825[1] = g21*(d_lidar - h) + g22*(-a_dyn + az - g) + v;
   out_8699371518319484825[2] = a_dyn + g31*(d_lidar - h) + g32*(-a_dyn + az - g);

}

void update_vertical_cov(float g11, float g12, float g21, float g22, float g31, float g32, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float *out_3791832019188488155) {

   out_3791832019188488155[0] = -g12*p31 + p11*(-g11 + 1);
   out_3791832019188488155[1] = -g12*p32 + p12*(-g11 + 1);
   out_3791832019188488155[2] = -g12*p33 + p13*(-g11 + 1);
   out_3791832019188488155[3] = -g21*p11 - g22*p31 + p21;
   out_3791832019188488155[4] = -g21*p12 - g22*p32 + p22;
   out_3791832019188488155[5] = -g21*p13 - g22*p33 + p23;
   out_3791832019188488155[6] = -g31*p11 + p31*(-g32 + 1);
   out_3791832019188488155[7] = -g31*p12 + p32*(-g32 + 1);
   out_3791832019188488155[8] = -g31*p13 + p33*(-g32 + 1);

}

void lidar_update_vertical_gain(float p11, float p21, float p31, float r_lidar, float *out_4466161268775116645) {

   out_4466161268775116645[0] = p11/(p11 + r_lidar);
   out_4466161268775116645[1] = p21/(p11 + r_lidar);
   out_4466161268775116645[2] = p31/(p11 + r_lidar);

}

void lidar_update_vertical_state(float a_dyn, float d_lidar, float g11, float g21, float g31, float h, float v, float *out_3494384365063633279) {

   out_3494384365063633279[0] = g11*(d_lidar - h) + h;
   out_3494384365063633279[1] = g21*(d_lidar - h) + v;
   out_3494384365063633279[2] = a_dyn + g31*(d_lidar - h);

}

void lidar_update_vertical_cov(float g11, float g21, float g31, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float *out_83866573131892437) {

   out_83866573131892437[0] = p11*(-g11 + 1);
   out_83866573131892437[1] = p12*(-g11 + 1);
   out_83866573131892437[2] = p13*(-g11 + 1);
   out_83866573131892437[3] = -g21*p11 + p21;
   out_83866573131892437[4] = -g21*p12 + p22;
   out_83866573131892437[5] = -g21*p13 + p23;
   out_83866573131892437[6] = -g31*p11 + p31;
   out_83866573131892437[7] = -g31*p12 + p32;
   out_83866573131892437[8] = -g31*p13 + p33;

}

void accel_update_vertical_gain(float p13, float p23, float p33, float r_accel, float *out_8346643870775845793) {

   out_8346643870775845793[0] = p13/(p33 + r_accel);
   out_8346643870775845793[1] = p23/(p33 + r_accel);
   out_8346643870775845793[2] = p33/(p33 + r_accel);

}

void accel_update_vertical_state(float a_dyn, float az, float g, float g11, float g21, float g31, float h, float v, float *out_4380166157680502389) {

   out_4380166157680502389[0] = g11*(-a_dyn + az - g) + h;
   out_4380166157680502389[1] = g21*(-a_dyn + az - g) + v;
   out_4380166157680502389[2] = a_dyn + g31*(-a_dyn + az - g);

}

void accel_update_vertical_cov(float g11, float g21, float g31, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float *out_2345034652353781171) {

   out_2345034652353781171[0] = -g11*p31 + p11;
   out_2345034652353781171[1] = -g11*p32 + p12;
   out_2345034652353781171[2] = -g11*p33 + p13;
   out_2345034652353781171[3] = -g21*p31 + p21;
   out_2345034652353781171[4] = -g21*p32 + p22;
   out_2345034652353781171[5] = -g21*p33 + p23;
   out_2345034652353781171[6] = p31*(-g31 + 1);
   out_2345034652353781171[7] = p32*(-g31 + 1);
   out_2345034652353781171[8] = p33*(-g31 + 1);

}
