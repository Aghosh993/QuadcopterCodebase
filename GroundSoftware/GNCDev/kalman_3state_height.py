#!/usr/bin/python3
from sympy import *
from sympy.utilities.codegen import codegen

r_sparse=True
q_sparse=True

def main():
	# Generic vars for predict,update steps:
	q11,q12,q13,q14,q21,q22,q23,q24,q31,q32,q33,q34,q41,q42,q43,q44 = symbols('q11,q12,q13,q14,q21,q22,q23,q24,q31,q32,q33,q34,q41,q42,q43,q44') # Process noise
	g11,g12,g13,g14,g21,g22,g23,g24,g31,g32,g33,g34,g41,g42,g43,g44 = symbols('g11,g12,g13,g14,g21,g22,g23,g24,g31,g32,g33,g34,g41,g42,g43,g44') # Kalman Gain
	p11,p12,p13,p14,p21,p22,p23,p24,p31,p32,p33,p34,p41,p42,p43,p44 = symbols('p11,p12,p13,p14,p21,p22,p23,p24,p31,p32,p33,p34,p41,p42,p43,p44') # Prediction covariance
	r11,r12,r13,r14,r21,r22,r23,r24,r31,r32,r33,r34,r41,r42,r43,r44 = symbols('r11,r12,r13,r14,r21,r22,r23,r24,r31,r32,r33,r34,r41,r42,r43,r44') # Sensor noise

	r_lidar,r_accel=symbols('r_lidar,r_accel')
	
	# Time between predict,update steps:
	dt=symbols('dt')
	
	# Gravity constant = 9.810 m*s^-1:
	g=symbols('g')

	# Observation:
	d_lidar,az=symbols('d_lidar,az')
	
	# State:
	h,v,a_dyn=symbols('h,v,a_dyn')


	A=Matrix([[1,dt,0.5*dt*dt],[0,1,dt],[0,0,1]])
	if q_sparse:
		print("Generating equations assuming sparse process noise matrix")
		Q=Matrix([[q11,0,0],[0,q22,0],[0,0,q33]])
	else:
		Q=Matrix([[q11,q12,q13],[q21,q22,q23],[q31,q32,q33]])

	P=Matrix([[p11,p12,p13],[p21,p22,p23],[p31,p32,p33]])
	G=Matrix([[g11,g12],[g21,g22],[g31,g32]])
	G_lidar_accel=Matrix([[g11],[g21],[g31]])
	H=Matrix([[1,0,0],[0,0,1]]) # Measurement matrix to correlate state to observation to find innovation
	H_lidar=Matrix([[1,0,0]])
	H_accel=Matrix([[0,0,1]])
	if r_sparse:
		R=Matrix([[r11,0],[0,r22]])
		R_lidar_M=Matrix([[r_lidar]])
		R_accel_M=Matrix([[r_accel]])
		print("Generating equations assuming sparse measurement noise matrix")
	else:
		R=Matrix([[r11,r12],[r21,r22]])
	
	z=Matrix([[d_lidar],[az-g]]) # Observation vector (overall)
	z_lidar=Matrix([[d_lidar]])
	z_accel=Matrix([[az-g]])

	x=Matrix([[h],[v],[a_dyn]]) # State vector

	expr=[]
	# Prediction equations:
	expr.append(A*x)
	expr.append(A*P*A.T + Q)

	# Update equations for observation vector z = [h_lidar,accel_z-g]:
	expr.append(P*H.T*(H*P*H.T + R).inverse_ADJ())
	expr.append(x + G*(z-H*x))
	expr.append((eye(3) - G*H)*P)

	# Update equations for observation vector z = [h_lidar]:
	expr.append(P*H_lidar.T*(H_lidar*P*H_lidar.T + R_lidar_M).inverse_ADJ())
	expr.append(x + G_lidar_accel*(z_lidar-H_lidar*x))
	expr.append((eye(3) - G_lidar_accel*H_lidar)*P)

	# Update equations for observation vector z = [accel_z-g]:
	expr.append(P*H_accel.T*(H_accel*P*H_accel.T + R_accel_M).inverse_ADJ())
	expr.append(x + G_lidar_accel*(z_accel-H_accel*x))
	expr.append((eye(3) - G_lidar_accel*H_accel)*P)

	[(c_name, c_code), (h_name, c_header)] = codegen((('predict_vertical_state', expr[0]),
														('predict_vertical_cov', expr[1]),
														
														('update_vertical_gain', expr[2]),
														('update_vertical_state', expr[3]),
														('update_vertical_cov', expr[4]),
														
														('lidar_update_vertical_gain', expr[5]),
														('lidar_update_vertical_state', expr[6]),
														('lidar_update_vertical_cov', expr[7]),
														
														('accel_update_vertical_gain', expr[8]),
														('accel_update_vertical_state', expr[9]),
														('accel_update_vertical_cov', expr[10])), "C", 'kalman_3state_height_autogen_test',project='Kalman_3state_height_Core')

	src=open(c_name, "w+")
	src.write(c_code)	
	src.close()

	inc=open(h_name, "w+")
	inc.write(c_header)
	inc.close()

if __name__ == '__main__':
	main()