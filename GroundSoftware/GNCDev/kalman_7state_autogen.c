/******************************************************************************
 *                       Code generated with sympy 1.0                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                 This file is part of 'Kalman_7state_Core'                  *
 ******************************************************************************/
#include "kalman_7state_autogen.h"
#include <math.h>

void predict_state7(float az_dyn, float dt, float h, float pitch, float pitch_rate, float roll, float roll_rate, float v, float *out_7294160474070428019) {

   out_7294160474070428019[0] = dt*roll_rate + roll;
   out_7294160474070428019[1] = dt*pitch_rate + pitch;
   out_7294160474070428019[2] = roll_rate;
   out_7294160474070428019[3] = pitch_rate;
   out_7294160474070428019[4] = 0.5*az_dyn*pow(dt, 2) + dt*v + h;
   out_7294160474070428019[5] = az_dyn*dt + v;
   out_7294160474070428019[6] = az_dyn;

}

void predict_cov7(float dt, float p11, float p12, float p13, float p14, float p15, float p16, float p17, float p21, float p22, float p23, float p24, float p25, float p26, float p27, float p31, float p32, float p33, float p34, float p35, float p36, float p37, float p41, float p42, float p43, float p44, float p45, float p46, float p47, float p51, float p52, float p53, float p54, float p55, float p56, float p57, float p61, float p62, float p63, float p64, float p65, float p66, float p67, float p71, float p72, float p73, float p74, float p75, float p76, float p77, float q11, float q22, float q33, float q44, float q55, float q66, float q77, float *out_1954641373464739435) {

   out_1954641373464739435[0] = dt*p31 + dt*(dt*p33 + p13) + p11 + q11;
   out_1954641373464739435[1] = dt*p32 + dt*(dt*p34 + p14) + p12;
   out_1954641373464739435[2] = dt*p33 + p13;
   out_1954641373464739435[3] = dt*p34 + p14;
   out_1954641373464739435[4] = 0.5*pow(dt, 2)*(dt*p37 + p17) + dt*p35 + dt*(dt*p36 + p16) + p15;
   out_1954641373464739435[5] = dt*p36 + dt*(dt*p37 + p17) + p16;
   out_1954641373464739435[6] = dt*p37 + p17;
   out_1954641373464739435[7] = dt*p41 + dt*(dt*p43 + p23) + p21;
   out_1954641373464739435[8] = dt*p42 + dt*(dt*p44 + p24) + p22 + q22;
   out_1954641373464739435[9] = dt*p43 + p23;
   out_1954641373464739435[10] = dt*p44 + p24;
   out_1954641373464739435[11] = 0.5*pow(dt, 2)*(dt*p47 + p27) + dt*p45 + dt*(dt*p46 + p26) + p25;
   out_1954641373464739435[12] = dt*p46 + dt*(dt*p47 + p27) + p26;
   out_1954641373464739435[13] = dt*p47 + p27;
   out_1954641373464739435[14] = dt*p33 + p31;
   out_1954641373464739435[15] = dt*p34 + p32;
   out_1954641373464739435[16] = p33 + q33;
   out_1954641373464739435[17] = p34;
   out_1954641373464739435[18] = 0.5*pow(dt, 2)*p37 + dt*p36 + p35;
   out_1954641373464739435[19] = dt*p37 + p36;
   out_1954641373464739435[20] = p37;
   out_1954641373464739435[21] = dt*p43 + p41;
   out_1954641373464739435[22] = dt*p44 + p42;
   out_1954641373464739435[23] = p43;
   out_1954641373464739435[24] = p44 + q44;
   out_1954641373464739435[25] = 0.5*pow(dt, 2)*p47 + dt*p46 + p45;
   out_1954641373464739435[26] = dt*p47 + p46;
   out_1954641373464739435[27] = p47;
   out_1954641373464739435[28] = 0.5*pow(dt, 2)*p71 + dt*p61 + dt*(0.5*pow(dt, 2)*p73 + dt*p63 + p53) + p51;
   out_1954641373464739435[29] = 0.5*pow(dt, 2)*p72 + dt*p62 + dt*(0.5*pow(dt, 2)*p74 + dt*p64 + p54) + p52;
   out_1954641373464739435[30] = 0.5*pow(dt, 2)*p73 + dt*p63 + p53;
   out_1954641373464739435[31] = 0.5*pow(dt, 2)*p74 + dt*p64 + p54;
   out_1954641373464739435[32] = 0.5*pow(dt, 2)*p75 + 0.5*pow(dt, 2)*(0.5*pow(dt, 2)*p77 + dt*p67 + p57) + dt*p65 + dt*(0.5*pow(dt, 2)*p76 + dt*p66 + p56) + p55 + q55;
   out_1954641373464739435[33] = 0.5*pow(dt, 2)*p76 + dt*p66 + dt*(0.5*pow(dt, 2)*p77 + dt*p67 + p57) + p56;
   out_1954641373464739435[34] = 0.5*pow(dt, 2)*p77 + dt*p67 + p57;
   out_1954641373464739435[35] = dt*p71 + dt*(dt*p73 + p63) + p61;
   out_1954641373464739435[36] = dt*p72 + dt*(dt*p74 + p64) + p62;
   out_1954641373464739435[37] = dt*p73 + p63;
   out_1954641373464739435[38] = dt*p74 + p64;
   out_1954641373464739435[39] = 0.5*pow(dt, 2)*(dt*p77 + p67) + dt*p75 + dt*(dt*p76 + p66) + p65;
   out_1954641373464739435[40] = dt*p76 + dt*(dt*p77 + p67) + p66 + q66;
   out_1954641373464739435[41] = dt*p77 + p67;
   out_1954641373464739435[42] = dt*p73 + p71;
   out_1954641373464739435[43] = dt*p74 + p72;
   out_1954641373464739435[44] = p73;
   out_1954641373464739435[45] = p74;
   out_1954641373464739435[46] = 0.5*pow(dt, 2)*p77 + dt*p76 + p75;
   out_1954641373464739435[47] = dt*p77 + p76;
   out_1954641373464739435[48] = p77 + q77;

}

void update_gain_accel(float g, float p11, float p12, float p17, float p21, float p22, float p27, float p31, float p32, float p37, float p41, float p42, float p47, float p51, float p52, float p57, float p61, float p62, float p67, float p71, float p72, float p77, float r11, float r22, float r33, float *out_3116608753311902538) {

   out_3116608753311902538[0] = g*p11*(-g*g*p12*(-p77 - r33) - g*g*p17*p72)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p12*(-g*g*p17*p71 + (-p77 - r33)*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p17*(-g*g*g*p12*p71 + g*p72*(g*g*p11 + r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[1] = g*p11*(-g*g*p27*p72 + (-p77 - r33)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p12*(-g*g*p21*(-p77 - r33) - g*g*p27*p71)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p17*(g*g*g*p21*p72 + g*p71*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[2] = g*p11*(g*g*g*p12*p27 + g*p17*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p12*(-g*g*g*p17*p21 - g*p27*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p17*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[3] = g*p21*(-g*g*p12*(-p77 - r33) - g*g*p17*p72)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p22*(-g*g*p17*p71 + (-p77 - r33)*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p27*(-g*g*g*p12*p71 + g*p72*(g*g*p11 + r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[4] = g*p21*(-g*g*p27*p72 + (-p77 - r33)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p22*(-g*g*p21*(-p77 - r33) - g*g*p27*p71)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p27*(g*g*g*p21*p72 + g*p71*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[5] = g*p21*(g*g*g*p12*p27 + g*p17*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p22*(-g*g*g*p17*p21 - g*p27*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p27*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[6] = g*p31*(-g*g*p12*(-p77 - r33) - g*g*p17*p72)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p32*(-g*g*p17*p71 + (-p77 - r33)*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p37*(-g*g*g*p12*p71 + g*p72*(g*g*p11 + r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[7] = g*p31*(-g*g*p27*p72 + (-p77 - r33)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p32*(-g*g*p21*(-p77 - r33) - g*g*p27*p71)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p37*(g*g*g*p21*p72 + g*p71*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[8] = g*p31*(g*g*g*p12*p27 + g*p17*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p32*(-g*g*g*p17*p21 - g*p27*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p37*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[9] = g*p41*(-g*g*p12*(-p77 - r33) - g*g*p17*p72)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p42*(-g*g*p17*p71 + (-p77 - r33)*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p47*(-g*g*g*p12*p71 + g*p72*(g*g*p11 + r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[10] = g*p41*(-g*g*p27*p72 + (-p77 - r33)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p42*(-g*g*p21*(-p77 - r33) - g*g*p27*p71)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p47*(g*g*g*p21*p72 + g*p71*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[11] = g*p41*(g*g*g*p12*p27 + g*p17*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p42*(-g*g*g*p17*p21 - g*p27*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p47*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[12] = g*p51*(-g*g*p12*(-p77 - r33) - g*g*p17*p72)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p52*(-g*g*p17*p71 + (-p77 - r33)*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p57*(-g*g*g*p12*p71 + g*p72*(g*g*p11 + r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[13] = g*p51*(-g*g*p27*p72 + (-p77 - r33)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p52*(-g*g*p21*(-p77 - r33) - g*g*p27*p71)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p57*(g*g*g*p21*p72 + g*p71*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[14] = g*p51*(g*g*g*p12*p27 + g*p17*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p52*(-g*g*g*p17*p21 - g*p27*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p57*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[15] = g*p61*(-g*g*p12*(-p77 - r33) - g*g*p17*p72)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p62*(-g*g*p17*p71 + (-p77 - r33)*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p67*(-g*g*g*p12*p71 + g*p72*(g*g*p11 + r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[16] = g*p61*(-g*g*p27*p72 + (-p77 - r33)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p62*(-g*g*p21*(-p77 - r33) - g*g*p27*p71)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p67*(g*g*g*p21*p72 + g*p71*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[17] = g*p61*(g*g*g*p12*p27 + g*p17*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p62*(-g*g*g*p17*p21 - g*p27*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p67*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[18] = g*p71*(-g*g*p12*(-p77 - r33) - g*g*p17*p72)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p72*(-g*g*p17*p71 + (-p77 - r33)*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p77*(-g*g*g*p12*p71 + g*p72*(g*g*p11 + r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[19] = g*p71*(-g*g*p27*p72 + (-p77 - r33)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p72*(-g*g*p21*(-p77 - r33) - g*g*p27*p71)/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p77*(g*g*g*p21*p72 + g*p71*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));
   out_3116608753311902538[20] = g*p71*(g*g*g*p12*p27 + g*p17*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(-g*g*p11 - r22))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22)) + p77*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11))/(g*p71*(g*g*g*p12*p27 + g*p17*(g*g*p11 + r22)) - g*p72*(-g*g*g*p17*p21 - g*p27*(g*g*p22 + r11)) - (-p77 - r33)*(-g*g*g*g*p12*p21 + (-g*g*p11 - r22)*(-g*g*p22 - r11)) - (-g*g*p17*p71 - g*g*p27*p72)*(-g*g*p11 - g*g*p22 - r11 - r22));

}

void update_innovation_accel(float ax, float ay, float az, float az_dyn, float g, float pitch, float roll, float *out_6225598155390527033) {

   out_6225598155390527033[0] = ax + g*pitch;
   out_6225598155390527033[1] = ay - g*roll;
   out_6225598155390527033[2] = az - az_dyn - g;

}

void update_state_accel(float ax, float ay, float az, float az_dyn, float g, float g11, float g12, float g13, float g21, float g22, float g23, float g31, float g32, float g33, float g41, float g42, float g43, float g51, float g52, float g53, float g61, float g62, float g63, float g71, float g72, float g73, float h, float pitch, float pitch_rate, float roll, float roll_rate, float v, float *out_5918412186688097944) {

   out_5918412186688097944[0] = g11*(ax + g*pitch) + g12*(ay - g*roll) + g13*(az - az_dyn - g) + roll;
   out_5918412186688097944[1] = g21*(ax + g*pitch) + g22*(ay - g*roll) + g23*(az - az_dyn - g) + pitch;
   out_5918412186688097944[2] = g31*(ax + g*pitch) + g32*(ay - g*roll) + g33*(az - az_dyn - g) + roll_rate;
   out_5918412186688097944[3] = g41*(ax + g*pitch) + g42*(ay - g*roll) + g43*(az - az_dyn - g) + pitch_rate;
   out_5918412186688097944[4] = g51*(ax + g*pitch) + g52*(ay - g*roll) + g53*(az - az_dyn - g) + h;
   out_5918412186688097944[5] = g61*(ax + g*pitch) + g62*(ay - g*roll) + g63*(az - az_dyn - g) + v;
   out_5918412186688097944[6] = az_dyn + g71*(ax + g*pitch) + g72*(ay - g*roll) + g73*(az - az_dyn - g);

}

void update_cov_accel(float g, float g11, float g12, float g13, float g21, float g22, float g23, float g31, float g32, float g33, float g41, float g42, float g43, float g51, float g52, float g53, float g61, float g62, float g63, float g71, float g72, float g73, float p11, float p12, float p13, float p14, float p15, float p16, float p17, float p21, float p22, float p23, float p24, float p25, float p26, float p27, float p31, float p32, float p33, float p34, float p35, float p36, float p37, float p41, float p42, float p43, float p44, float p45, float p46, float p47, float p51, float p52, float p53, float p54, float p55, float p56, float p57, float p61, float p62, float p63, float p64, float p65, float p66, float p67, float p71, float p72, float p73, float p74, float p75, float p76, float p77, float *out_8899364086708404823) {

   out_8899364086708404823[0] = g*g11*p21 - g13*p71 + p11*(-g*g12 + 1);
   out_8899364086708404823[1] = g*g11*p22 - g13*p72 + p12*(-g*g12 + 1);
   out_8899364086708404823[2] = g*g11*p23 - g13*p73 + p13*(-g*g12 + 1);
   out_8899364086708404823[3] = g*g11*p24 - g13*p74 + p14*(-g*g12 + 1);
   out_8899364086708404823[4] = g*g11*p25 - g13*p75 + p15*(-g*g12 + 1);
   out_8899364086708404823[5] = g*g11*p26 - g13*p76 + p16*(-g*g12 + 1);
   out_8899364086708404823[6] = g*g11*p27 - g13*p77 + p17*(-g*g12 + 1);
   out_8899364086708404823[7] = -g*g22*p11 - g23*p71 + p21*(g*g21 + 1);
   out_8899364086708404823[8] = -g*g22*p12 - g23*p72 + p22*(g*g21 + 1);
   out_8899364086708404823[9] = -g*g22*p13 - g23*p73 + p23*(g*g21 + 1);
   out_8899364086708404823[10] = -g*g22*p14 - g23*p74 + p24*(g*g21 + 1);
   out_8899364086708404823[11] = -g*g22*p15 - g23*p75 + p25*(g*g21 + 1);
   out_8899364086708404823[12] = -g*g22*p16 - g23*p76 + p26*(g*g21 + 1);
   out_8899364086708404823[13] = -g*g22*p17 - g23*p77 + p27*(g*g21 + 1);
   out_8899364086708404823[14] = g*g31*p21 - g*g32*p11 - g33*p71 + p31;
   out_8899364086708404823[15] = g*g31*p22 - g*g32*p12 - g33*p72 + p32;
   out_8899364086708404823[16] = g*g31*p23 - g*g32*p13 - g33*p73 + p33;
   out_8899364086708404823[17] = g*g31*p24 - g*g32*p14 - g33*p74 + p34;
   out_8899364086708404823[18] = g*g31*p25 - g*g32*p15 - g33*p75 + p35;
   out_8899364086708404823[19] = g*g31*p26 - g*g32*p16 - g33*p76 + p36;
   out_8899364086708404823[20] = g*g31*p27 - g*g32*p17 - g33*p77 + p37;
   out_8899364086708404823[21] = g*g41*p21 - g*g42*p11 - g43*p71 + p41;
   out_8899364086708404823[22] = g*g41*p22 - g*g42*p12 - g43*p72 + p42;
   out_8899364086708404823[23] = g*g41*p23 - g*g42*p13 - g43*p73 + p43;
   out_8899364086708404823[24] = g*g41*p24 - g*g42*p14 - g43*p74 + p44;
   out_8899364086708404823[25] = g*g41*p25 - g*g42*p15 - g43*p75 + p45;
   out_8899364086708404823[26] = g*g41*p26 - g*g42*p16 - g43*p76 + p46;
   out_8899364086708404823[27] = g*g41*p27 - g*g42*p17 - g43*p77 + p47;
   out_8899364086708404823[28] = g*g51*p21 - g*g52*p11 - g53*p71 + p51;
   out_8899364086708404823[29] = g*g51*p22 - g*g52*p12 - g53*p72 + p52;
   out_8899364086708404823[30] = g*g51*p23 - g*g52*p13 - g53*p73 + p53;
   out_8899364086708404823[31] = g*g51*p24 - g*g52*p14 - g53*p74 + p54;
   out_8899364086708404823[32] = g*g51*p25 - g*g52*p15 - g53*p75 + p55;
   out_8899364086708404823[33] = g*g51*p26 - g*g52*p16 - g53*p76 + p56;
   out_8899364086708404823[34] = g*g51*p27 - g*g52*p17 - g53*p77 + p57;
   out_8899364086708404823[35] = g*g61*p21 - g*g62*p11 - g63*p71 + p61;
   out_8899364086708404823[36] = g*g61*p22 - g*g62*p12 - g63*p72 + p62;
   out_8899364086708404823[37] = g*g61*p23 - g*g62*p13 - g63*p73 + p63;
   out_8899364086708404823[38] = g*g61*p24 - g*g62*p14 - g63*p74 + p64;
   out_8899364086708404823[39] = g*g61*p25 - g*g62*p15 - g63*p75 + p65;
   out_8899364086708404823[40] = g*g61*p26 - g*g62*p16 - g63*p76 + p66;
   out_8899364086708404823[41] = g*g61*p27 - g*g62*p17 - g63*p77 + p67;
   out_8899364086708404823[42] = g*g71*p21 - g*g72*p11 + p71*(-g73 + 1);
   out_8899364086708404823[43] = g*g71*p22 - g*g72*p12 + p72*(-g73 + 1);
   out_8899364086708404823[44] = g*g71*p23 - g*g72*p13 + p73*(-g73 + 1);
   out_8899364086708404823[45] = g*g71*p24 - g*g72*p14 + p74*(-g73 + 1);
   out_8899364086708404823[46] = g*g71*p25 - g*g72*p15 + p75*(-g73 + 1);
   out_8899364086708404823[47] = g*g71*p26 - g*g72*p16 + p76*(-g73 + 1);
   out_8899364086708404823[48] = g*g71*p27 - g*g72*p17 + p77*(-g73 + 1);

}

void update_gain_gyro(float p13, float p14, float p23, float p24, float p33, float p34, float p43, float p44, float p53, float p54, float p63, float p64, float p73, float p74, float r44, float r55, float *out_8391242881630657245) {

   out_8391242881630657245[0] = p13*(p44 + r55)/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) - p14*p43/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[1] = -p13*p34/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p14*(p33 + r44)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[2] = p23*(p44 + r55)/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) - p24*p43/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[3] = -p23*p34/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p24*(p33 + r44)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[4] = p33*(p44 + r55)/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) - p34*p43/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[5] = -p33*p34/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p34*(p33 + r44)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[6] = -p43*p44/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p43*(p44 + r55)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[7] = -p34*p43/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p44*(p33 + r44)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[8] = -p43*p54/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p53*(p44 + r55)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[9] = -p34*p53/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p54*(p33 + r44)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[10] = -p43*p64/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p63*(p44 + r55)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[11] = -p34*p63/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p64*(p33 + r44)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[12] = -p43*p74/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p73*(p44 + r55)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));
   out_8391242881630657245[13] = -p34*p73/(-p34*p43 + (-p33 - r44)*(-p44 - r55)) + p74*(p33 + r44)/(-p34*p43 + (-p33 - r44)*(-p44 - r55));

}

void update_innovation_gyro(float gyro_pitch, float gyro_roll, float pitch_rate, float roll_rate, float *out_5217011749530427604) {

   out_5217011749530427604[0] = gyro_roll - roll_rate;
   out_5217011749530427604[1] = gyro_pitch - pitch_rate;

}

void update_state_gyro(float az_dyn, float g11, float g12, float g21, float g22, float g31, float g32, float g41, float g42, float g51, float g52, float g61, float g62, float g71, float g72, float gyro_pitch, float gyro_roll, float h, float pitch, float pitch_rate, float roll, float roll_rate, float v, float *out_2100921197882637313) {

   out_2100921197882637313[0] = g11*(gyro_roll - roll_rate) + g12*(gyro_pitch - pitch_rate) + roll;
   out_2100921197882637313[1] = g21*(gyro_roll - roll_rate) + g22*(gyro_pitch - pitch_rate) + pitch;
   out_2100921197882637313[2] = g31*(gyro_roll - roll_rate) + g32*(gyro_pitch - pitch_rate) + roll_rate;
   out_2100921197882637313[3] = g41*(gyro_roll - roll_rate) + g42*(gyro_pitch - pitch_rate) + pitch_rate;
   out_2100921197882637313[4] = g51*(gyro_roll - roll_rate) + g52*(gyro_pitch - pitch_rate) + h;
   out_2100921197882637313[5] = g61*(gyro_roll - roll_rate) + g62*(gyro_pitch - pitch_rate) + v;
   out_2100921197882637313[6] = az_dyn + g71*(gyro_roll - roll_rate) + g72*(gyro_pitch - pitch_rate);

}

void update_cov_gyro(float g11, float g12, float g21, float g22, float g31, float g32, float g41, float g42, float g51, float g52, float g61, float g62, float g71, float g72, float p11, float p12, float p13, float p14, float p15, float p16, float p17, float p21, float p22, float p23, float p24, float p25, float p26, float p27, float p31, float p32, float p33, float p34, float p35, float p36, float p37, float p41, float p42, float p43, float p44, float p45, float p46, float p47, float p51, float p52, float p53, float p54, float p55, float p56, float p57, float p61, float p62, float p63, float p64, float p65, float p66, float p67, float p71, float p72, float p73, float p74, float p75, float p76, float p77, float *out_2632967519352958486) {

   out_2632967519352958486[0] = -g11*p31 - g12*p41 + p11;
   out_2632967519352958486[1] = -g11*p32 - g12*p42 + p12;
   out_2632967519352958486[2] = -g11*p33 - g12*p43 + p13;
   out_2632967519352958486[3] = -g11*p34 - g12*p44 + p14;
   out_2632967519352958486[4] = -g11*p35 - g12*p45 + p15;
   out_2632967519352958486[5] = -g11*p36 - g12*p46 + p16;
   out_2632967519352958486[6] = -g11*p37 - g12*p47 + p17;
   out_2632967519352958486[7] = -g21*p31 - g22*p41 + p21;
   out_2632967519352958486[8] = -g21*p32 - g22*p42 + p22;
   out_2632967519352958486[9] = -g21*p33 - g22*p43 + p23;
   out_2632967519352958486[10] = -g21*p34 - g22*p44 + p24;
   out_2632967519352958486[11] = -g21*p35 - g22*p45 + p25;
   out_2632967519352958486[12] = -g21*p36 - g22*p46 + p26;
   out_2632967519352958486[13] = -g21*p37 - g22*p47 + p27;
   out_2632967519352958486[14] = -g32*p41 + p31*(-g31 + 1);
   out_2632967519352958486[15] = -g32*p42 + p32*(-g31 + 1);
   out_2632967519352958486[16] = -g32*p43 + p33*(-g31 + 1);
   out_2632967519352958486[17] = -g32*p44 + p34*(-g31 + 1);
   out_2632967519352958486[18] = -g32*p45 + p35*(-g31 + 1);
   out_2632967519352958486[19] = -g32*p46 + p36*(-g31 + 1);
   out_2632967519352958486[20] = -g32*p47 + p37*(-g31 + 1);
   out_2632967519352958486[21] = -g41*p31 + p41*(-g42 + 1);
   out_2632967519352958486[22] = -g41*p32 + p42*(-g42 + 1);
   out_2632967519352958486[23] = -g41*p33 + p43*(-g42 + 1);
   out_2632967519352958486[24] = -g41*p34 + p44*(-g42 + 1);
   out_2632967519352958486[25] = -g41*p35 + p45*(-g42 + 1);
   out_2632967519352958486[26] = -g41*p36 + p46*(-g42 + 1);
   out_2632967519352958486[27] = -g41*p37 + p47*(-g42 + 1);
   out_2632967519352958486[28] = -g51*p31 - g52*p41 + p51;
   out_2632967519352958486[29] = -g51*p32 - g52*p42 + p52;
   out_2632967519352958486[30] = -g51*p33 - g52*p43 + p53;
   out_2632967519352958486[31] = -g51*p34 - g52*p44 + p54;
   out_2632967519352958486[32] = -g51*p35 - g52*p45 + p55;
   out_2632967519352958486[33] = -g51*p36 - g52*p46 + p56;
   out_2632967519352958486[34] = -g51*p37 - g52*p47 + p57;
   out_2632967519352958486[35] = -g61*p31 - g62*p41 + p61;
   out_2632967519352958486[36] = -g61*p32 - g62*p42 + p62;
   out_2632967519352958486[37] = -g61*p33 - g62*p43 + p63;
   out_2632967519352958486[38] = -g61*p34 - g62*p44 + p64;
   out_2632967519352958486[39] = -g61*p35 - g62*p45 + p65;
   out_2632967519352958486[40] = -g61*p36 - g62*p46 + p66;
   out_2632967519352958486[41] = -g61*p37 - g62*p47 + p67;
   out_2632967519352958486[42] = -g71*p31 - g72*p41 + p71;
   out_2632967519352958486[43] = -g71*p32 - g72*p42 + p72;
   out_2632967519352958486[44] = -g71*p33 - g72*p43 + p73;
   out_2632967519352958486[45] = -g71*p34 - g72*p44 + p74;
   out_2632967519352958486[46] = -g71*p35 - g72*p45 + p75;
   out_2632967519352958486[47] = -g71*p36 - g72*p46 + p76;
   out_2632967519352958486[48] = -g71*p37 - g72*p47 + p77;

}

void update_gain_rangefinder(float p15, float p25, float p35, float p45, float p55, float p65, float p75, float r66, float *out_1463115039308031161) {

   out_1463115039308031161[0] = p15/(p55 + r66);
   out_1463115039308031161[1] = p25/(p55 + r66);
   out_1463115039308031161[2] = p35/(p55 + r66);
   out_1463115039308031161[3] = p45/(p55 + r66);
   out_1463115039308031161[4] = p55/(p55 + r66);
   out_1463115039308031161[5] = p65/(p55 + r66);
   out_1463115039308031161[6] = p75/(p55 + r66);

}

void update_innovation_rangefinder(float h, float h_lidar, float *out_5326467223387434636) {

   out_5326467223387434636[0] = -h + h_lidar;

}

void update_state_rangefinder(float az_dyn, float g11, float g21, float g31, float g41, float g51, float g61, float g71, float h, float h_lidar, float pitch, float pitch_rate, float roll, float roll_rate, float v, float *out_7697559358414380243) {

   out_7697559358414380243[0] = g11*(-h + h_lidar) + roll;
   out_7697559358414380243[1] = g21*(-h + h_lidar) + pitch;
   out_7697559358414380243[2] = g31*(-h + h_lidar) + roll_rate;
   out_7697559358414380243[3] = g41*(-h + h_lidar) + pitch_rate;
   out_7697559358414380243[4] = g51*(-h + h_lidar) + h;
   out_7697559358414380243[5] = g61*(-h + h_lidar) + v;
   out_7697559358414380243[6] = az_dyn + g71*(-h + h_lidar);

}

void update_cov_rangefinder(float g11, float g21, float g31, float g41, float g51, float g61, float g71, float p11, float p12, float p13, float p14, float p15, float p16, float p17, float p21, float p22, float p23, float p24, float p25, float p26, float p27, float p31, float p32, float p33, float p34, float p35, float p36, float p37, float p41, float p42, float p43, float p44, float p45, float p46, float p47, float p51, float p52, float p53, float p54, float p55, float p56, float p57, float p61, float p62, float p63, float p64, float p65, float p66, float p67, float p71, float p72, float p73, float p74, float p75, float p76, float p77, float *out_4405977212521852574) {

   out_4405977212521852574[0] = -g11*p51 + p11;
   out_4405977212521852574[1] = -g11*p52 + p12;
   out_4405977212521852574[2] = -g11*p53 + p13;
   out_4405977212521852574[3] = -g11*p54 + p14;
   out_4405977212521852574[4] = -g11*p55 + p15;
   out_4405977212521852574[5] = -g11*p56 + p16;
   out_4405977212521852574[6] = -g11*p57 + p17;
   out_4405977212521852574[7] = -g21*p51 + p21;
   out_4405977212521852574[8] = -g21*p52 + p22;
   out_4405977212521852574[9] = -g21*p53 + p23;
   out_4405977212521852574[10] = -g21*p54 + p24;
   out_4405977212521852574[11] = -g21*p55 + p25;
   out_4405977212521852574[12] = -g21*p56 + p26;
   out_4405977212521852574[13] = -g21*p57 + p27;
   out_4405977212521852574[14] = -g31*p51 + p31;
   out_4405977212521852574[15] = -g31*p52 + p32;
   out_4405977212521852574[16] = -g31*p53 + p33;
   out_4405977212521852574[17] = -g31*p54 + p34;
   out_4405977212521852574[18] = -g31*p55 + p35;
   out_4405977212521852574[19] = -g31*p56 + p36;
   out_4405977212521852574[20] = -g31*p57 + p37;
   out_4405977212521852574[21] = -g41*p51 + p41;
   out_4405977212521852574[22] = -g41*p52 + p42;
   out_4405977212521852574[23] = -g41*p53 + p43;
   out_4405977212521852574[24] = -g41*p54 + p44;
   out_4405977212521852574[25] = -g41*p55 + p45;
   out_4405977212521852574[26] = -g41*p56 + p46;
   out_4405977212521852574[27] = -g41*p57 + p47;
   out_4405977212521852574[28] = p51*(-g51 + 1);
   out_4405977212521852574[29] = p52*(-g51 + 1);
   out_4405977212521852574[30] = p53*(-g51 + 1);
   out_4405977212521852574[31] = p54*(-g51 + 1);
   out_4405977212521852574[32] = p55*(-g51 + 1);
   out_4405977212521852574[33] = p56*(-g51 + 1);
   out_4405977212521852574[34] = p57*(-g51 + 1);
   out_4405977212521852574[35] = -g61*p51 + p61;
   out_4405977212521852574[36] = -g61*p52 + p62;
   out_4405977212521852574[37] = -g61*p53 + p63;
   out_4405977212521852574[38] = -g61*p54 + p64;
   out_4405977212521852574[39] = -g61*p55 + p65;
   out_4405977212521852574[40] = -g61*p56 + p66;
   out_4405977212521852574[41] = -g61*p57 + p67;
   out_4405977212521852574[42] = -g71*p51 + p71;
   out_4405977212521852574[43] = -g71*p52 + p72;
   out_4405977212521852574[44] = -g71*p53 + p73;
   out_4405977212521852574[45] = -g71*p54 + p74;
   out_4405977212521852574[46] = -g71*p55 + p75;
   out_4405977212521852574[47] = -g71*p56 + p76;
   out_4405977212521852574[48] = -g71*p57 + p77;

}
