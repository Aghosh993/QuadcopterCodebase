#!/usr/bin/python3
from sympy import *
from sympy.utilities.codegen import codegen

# BSD 3-Clause License

# Copyright (c) 2017, Abhimanyu Ghosh
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.

# * Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This is a script that generates C code for a 7-state Linear Kalman Filter (KF)
# The filter estimates roll, pitch, body-fixed roll and pitch rates, body-fixed height, vertical velocity and vertical dynamic acceleration using a 6-axis IMU (gyro+accelerometer),
# and some kind of rangefinder (typically laser)
# The measurements are assumed to be body-aligned (in a local NED mody frame that is typical on aircraft platforms) with a right-hand coordinate system
# The accelerometer must indicate a raw Z value of +9.810 m*s^-2 when at rest and Z is pointing straight down.
# Gyro data is expected to be provided in radians*sec^-1
# All states are provided in scientific/SI units. I.e. displacements in meters, velocities in m*s^-1, accelerations in m*s^-2, angles and angular rates in radians and radians*sec^-1
# When the C code is used in the user applications, it is nominally expected that the filter always propagate the state vector forward using the predict() functions
# Additionally, whenever new observations are available, the appropriate measurement update() functions shall be run to correct the stat vector and update the Kalman Gain and covariances

r_sparse=True
q_sparse=True

def main():
	# Vars for predict,update steps:
	q11,q12,q13,q14,q15,q16,q17,q21,q22,q23,q24,q25,q26,q27,q31,q32,q33,q34,q35,q36,q37,q41,q42,q43,q44,q45,q46,q47,q51,q52,q53,q54,q55,q56,q57,q61,q62,q63,q64,q65,q66,q67,q71,q72,q73,q74,q75,q76,q77 = symbols('q11,q12,q13,q14,q15,q16,q17,q21,q22,q23,q24,q25,q26,q27,q31,q32,q33,q34,q35,q36,q37,q41,q42,q43,q44,q45,q46,q47,q51,q52,q53,q54,q55,q56,q57,q61,q62,q63,q64,q65,q66,q67,q71,q72,q73,q74,q75,q76,q77') # Process noise
	g11,g12,g13,g14,g15,g16,g17,g21,g22,g23,g24,g25,g26,g27,g31,g32,g33,g34,g35,g36,g37,g41,g42,g43,g44,g45,g46,g47,g51,g52,g53,g54,g55,g56,g57,g61,g62,g63,g64,g65,g66,g67,g71,g72,g73,g74,g75,g76,g77 = symbols('g11,g12,g13,g14,g15,g16,g17,g21,g22,g23,g24,g25,g26,g27,g31,g32,g33,g34,g35,g36,g37,g41,g42,g43,g44,g45,g46,g47,g51,g52,g53,g54,g55,g56,g57,g61,g62,g63,g64,g65,g66,g67,g71,g72,g73,g74,g75,g76,g77') # Kalman Gain
	p11,p12,p13,p14,p15,p16,p17,p21,p22,p23,p24,p25,p26,p27,p31,p32,p33,p34,p35,p36,p37,p41,p42,p43,p44,p45,p46,p47,p51,p52,p53,p54,p55,p56,p57,p61,p62,p63,p64,p65,p66,p67,p71,p72,p73,p74,p75,p76,p77 = symbols('p11,p12,p13,p14,p15,p16,p17,p21,p22,p23,p24,p25,p26,p27,p31,p32,p33,p34,p35,p36,p37,p41,p42,p43,p44,p45,p46,p47,p51,p52,p53,p54,p55,p56,p57,p61,p62,p63,p64,p65,p66,p67,p71,p72,p73,p74,p75,p76,p77') # Prediction covariance
	r11,r12,r13,r14,r15,r16,r17,r21,r22,r23,r24,r25,r26,r27,r31,r32,r33,r34,r35,r36,r37,r41,r42,r43,r44,r45,r46,r47,r51,r52,r53,r54,r55,r56,r57,r61,r62,r63,r64,r65,r66,r67,r71,r72,r73,r74,r75,r76,r77 = symbols('r11,r12,r13,r14,r15,r16,r17,r21,r22,r23,r24,r25,r26,r27,r31,r32,r33,r34,r35,r36,r37,r41,r42,r43,r44,r45,r46,r47,r51,r52,r53,r54,r55,r56,r57,r61,r62,r63,r64,r65,r66,r67,r71,r72,r73,r74,r75,r76,r77') # Sensor noise
	
	# Time between predict,update steps:
	dt=symbols('dt')
	
	# Gravity constant = 9.810 m*s^-1:
	g=symbols('g')

	# Observation:
	ax,ay,az,gyro_roll,gyro_pitch,h_lidar=symbols('ax, ay, az, gyro_roll, gyro_pitch, h_lidar')
	
	# State:
	roll,pitch,roll_rate,pitch_rate,h,v,az_dyn=symbols('roll,pitch,roll_rate,pitch_rate,h,v,az_dyn')


	A=Matrix([[1,0,dt,0,0,0,0],
				[0,1,0,dt,0,0,0],
				[0,0,1,0,0,0,0],
				[0,0,0,1,0,0,0],
				[0,0,0,0,1,dt,0.5*dt*dt],
				[0,0,0,0,0,1,dt],
				[0,0,0,0,0,0,1]])
	if q_sparse: # Only use diagonal elements in the Q matrix since this is the typical model of process noise...
		print("Generating equations assuming sparse process noise matrix")
		Q=Matrix([[q11,0,0,0,0,0,0],[0,q22,0,0,0,0,0],[0,0,q33,0,0,0,0],[0,0,0,q44,0,0,0],[0,0,0,0,q55,0,0],[0,0,0,0,0,q66,0],[0,0,0,0,0,0,q77]])
	else:
		Q=Matrix([[q11,q12,q13,q14,q15,q16,q17],[q21,q22,q23,q24,q25,q26,q27],[q31,q32,q33,q34,q35,q36,q37],[q41,q42,q43,q44,q45,q46,q47],[q51,q52,q53,q54,q55,q56,q57],[q61,q62,q63,q64,q65,q66,q67],[q71,q72,q73,q74,q75,q76,q77]])

	P=Matrix([[p11,p12,p13,p14,p15,p16,p17],[p21,p22,p23,p24,p25,p26,p27],[p31,p32,p33,p34,p35,p36,p37],[p41,p42,p43,p44,p45,p46,p47],[p51,p52,p53,p54,p55,p56,p57],[p61,p62,p63,p64,p65,p66,p67],[p71,p72,p73,p74,p75,p76,p77]])
	
	G=Matrix([[g11,g12,g13,g14,g15,g16],[g21,g22,g23,g24,g25,g26],[g31,g32,g33,g34,g35,g36],[g41,g42,g43,g44,g45,g46],[g51,g52,g53,g54,g55,g56],[g61,g62,g63,g64,g65,g66],[g71,g72,g73,g74,g75,g76]])
	G_accel=Matrix([[g11,g12,g13],[g21,g22,g23],[g31,g32,g33],[g41,g42,g43],[g51,g52,g53],[g61,g62,g63],[g71,g72,g73]])
	G_gyro=Matrix([[g11,g12],[g21,g22],[g31,g32],[g41,g42],[g51,g52],[g61,g62],[g71,g72]])
	G_rangefinder=Matrix([[g11],[g21],[g31],[g41],[g51],[g61],[g71]])

	H=Matrix([[0,-g,0,0,0,0,0],
				[g,0,0,0,0,0,0],
				[0,0,0,0,0,0,1],
				[0,0,1,0,0,0,0],
				[0,0,0,1,0,0,0],
				[0,0,0,0,1,0,0]]) # Measurement matrix to correlate state to observation to find innovation
	
	H_accel=Matrix([[0,-g,0,0,0,0,0],
					[g,0,0,0,0,0,0],
					[0,0,0,0,0,0,1]])
	
	H_gyro=Matrix([[0,0,1,0,0,0,0],
					[0,0,0,1,0,0,0]])
	
	H_rangefinder=Matrix([[0,0,0,0,1,0,0]])	
	
	if r_sparse:
		print("Generating equations assuming sparse measurement noise matrix")
		
		R=Matrix([[r11,0,0,0,0,0],
					[0,r22,0,0,0,0],
					[0,0,r33,0,0,0],
					[0,0,0,r44,0,0],
					[0,0,0,0,r55,0],
					[0,0,0,0,0,r66]]) # Only assume diagonal nonzero elements of the measurement noise model...
		
		R_accel=Matrix([[r11,0,0],
						[0,r22,0],
						[0,0,r33]])
		
		R_gyro=Matrix([[r44,0],
						[0,r55]])
		
		R_rangefinder=Matrix([[r66]])		
	else:
		R=Matrix([[r11,r12,r13,r14,r15,r16],
					[r21,r22,r23,r24,r25,r26],
					[r31,r32,r33,r34,r35,r36],
					[r41,r42,r43,r44,r45,r46],
					[r51,r52,r53,r54,r55,r56],
					[r61,r62,r63,r64,r65,r66]])
	
	z=Matrix([[ax],
				[ay],
				[az-g],
				[gyro_roll],
				[gyro_pitch],
				[h_lidar]]) # Observation vector for all sensors at same time
	
	z_accel=Matrix([[ax],
					[ay],
					[az-g]])
	
	z_gyro=Matrix([[gyro_roll],
					[gyro_pitch]])
	
	z_rangefinder=Matrix([[h_lidar]])

	x=Matrix([[roll],
				[pitch],
				[roll_rate],
				[pitch_rate],
				[h],
				[v],
				[az_dyn]]) # State vector

	expr=[]

	# Prediction Equations:
	expr.append(A*x)
	expr.append(A*P*A.T + Q)

	# Update using accelerometer data:
	expr.append(P*H_accel.T*(H_accel*P*H_accel.T + R_accel).inverse_ADJ())
	expr.append(z_accel-H_accel*x)
	expr.append(x + G_accel*(z_accel-H_accel*x))
	expr.append((eye(7) - G_accel*H_accel)*P)

	# Update using gyro data:
	expr.append(P*H_gyro.T*(H_gyro*P*H_gyro.T + R_gyro).inverse_ADJ())
	expr.append(z_gyro-H_gyro*x)
	expr.append(x + G_gyro*(z_gyro-H_gyro*x))
	expr.append((eye(7) - G_gyro*H_gyro)*P)

	# Update using rangefinder data:
	expr.append(P*H_rangefinder.T*(H_rangefinder*P*H_rangefinder.T + R_rangefinder).inverse_ADJ())
	expr.append(z_rangefinder-H_rangefinder*x)
	expr.append(x + G_rangefinder*(z_rangefinder-H_rangefinder*x))
	expr.append((eye(7) - G_rangefinder*H_rangefinder)*P)

	[(c_name, c_code), (h_name, c_header)] = codegen((('predict_state7', expr[0]),
														('predict_cov7', expr[1]),

														('update_gain_accel', expr[2]),
														('update_innovation_accel', expr[3]),
														('update_state_accel', expr[4]),
														('update_cov_accel', expr[5]),

														('update_gain_gyro', expr[6]),
														('update_innovation_gyro', expr[7]),
														('update_state_gyro', expr[8]),
														('update_cov_gyro', expr[9]),

														('update_gain_rangefinder', expr[10]),
														('update_innovation_rangefinder', expr[11]),
														('update_state_rangefinder', expr[12]),
														('update_cov_rangefinder', expr[13])), "C", 'kalman_7state_autogen',project='Kalman_7state_Core')

	src=open(c_name, "w+")
	src.write(c_code)	
	src.close()

	inc=open(h_name, "w+")
	inc.write(c_header)
	inc.close()

if __name__ == '__main__':
	main()