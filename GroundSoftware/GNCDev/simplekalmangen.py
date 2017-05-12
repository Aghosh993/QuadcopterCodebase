#!/usr/bin/python3
from sympy import *
from sympy.utilities.codegen import codegen

r_sparse=True
q_sparse=True

def main():
	# Vars for predict,update steps:
	q11,q12,q13,q14,q21,q22,q23,q24,q31,q32,q33,q34,q41,q42,q43,q44 = symbols('q11,q12,q13,q14,q21,q22,q23,q24,q31,q32,q33,q34,q41,q42,q43,q44') # Process noise
	g11,g12,g13,g14,g21,g22,g23,g24,g31,g32,g33,g34,g41,g42,g43,g44 = symbols('g11,g12,g13,g14,g21,g22,g23,g24,g31,g32,g33,g34,g41,g42,g43,g44') # Kalman Gain
	p11,p12,p13,p14,p21,p22,p23,p24,p31,p32,p33,p34,p41,p42,p43,p44 = symbols('p11,p12,p13,p14,p21,p22,p23,p24,p31,p32,p33,p34,p41,p42,p43,p44') # Prediction covariance
	r11,r12,r13,r14,r21,r22,r23,r24,r31,r32,r33,r34,r41,r42,r43,r44 = symbols('r11,r12,r13,r14,r21,r22,r23,r24,r31,r32,r33,r34,r41,r42,r43,r44') # Sensor noise
	
	# Time between predict,update steps:
	dt=symbols('dt')
	
	# Gravity constant = 9.810 m*s^-1:
	g=symbols('g')

	# Observation:
	ax,ay,gyro_roll,gyro_pitch=symbols('ax, ay, gyro_roll, gyro_pitch')
	
	# State:
	roll,pitch,roll_rate,pitch_rate=symbols('roll,pitch,roll_rate,pitch_rate')


	A=Matrix([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
	if q_sparse:
		print("Generating equations assuming sparse process noise matrix")
		Q=Matrix([[q11,0,0,0],[0,q22,0,0],[0,0,q33,0],[0,0,0,q44]])
	else:
		Q=Matrix([[q11,q12,q13,q14],[q21,q22,q23,q24],[q31,q32,q33,q34],[q41,q42,q43,q44]])

	P=Matrix([[p11,p12,p13,p14],[p21,p22,p23,p24],[p31,p32,p33,p34],[p41,p42,p43,p44]])
	G=Matrix([[g11,g12,g13,g14],[g21,g22,g23,g24],[g31,g32,g33,g34],[g41,g42,g43,g44]])
	H=Matrix([[0,-g,0,0],[g,0,0,0],[0,0,1,0],[0,0,0,1]]) # Measurement matrix to correlate state to observation to find innovation
	if r_sparse:
		R=Matrix([[r11,0,0,0],[0,r22,0,0],[0,0,r33,0],[0,0,0,r44]])
		print("Generating equations assuming sparse measurement noise matrix")
	else:
		R=Matrix([[r11,r12,r13,r14],[r21,r22,r23,r24],[r31,r32,r33,r34],[r41,r42,r43,r44]])
	
	z=Matrix([[ax],[ay],[gyro_roll],[gyro_pitch]]) # Observation vector
	x=Matrix([[roll],[pitch],[roll_rate],[pitch_rate]]) # State vector

	expr=[]
	expr.append(A*x)
	expr.append(A*P*A.T + Q)
	expr.append(P*H.T*(H*P*H.T + R).inverse_ADJ())
	expr.append(x + G*(z-H*x))
	expr.append((eye(4) - G*H)*P)

	[(c_name, c_code), (h_name, c_header)] = codegen((('predict_state', expr[0]),
														('predict_cov', expr[1]),
														('update_gain', expr[2]),
														('update_state', expr[3]),
														('update_cov', expr[4])), "C", 'kalman_4state_autogen')

	src=open(c_name, "w+")
	src.write(c_code)	
	src.close()

	inc=open(h_name, "w+")
	inc.write(c_header)
	inc.close()

if __name__ == '__main__':
	main()