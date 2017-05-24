#!/usr/bin/python3
from sympy import *
from sympy.utilities.codegen import codegen

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


	A=Matrix([[1,0,dt,0,0,0,0],[0,1,0,dt,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0],[0,0,0,0,1,dt,dt*dt],[0,0,0,0,0,1,dt],[0,0,0,0,0,0,1]])
	if q_sparse:
		print("Generating equations assuming sparse process noise matrix")
		Q=Matrix([[q11,0,0,0,0,0,0],[0,q22,0,0,0,0,0],[0,0,q33,0,0,0,0],[0,0,0,q44,0,0,0],[0,0,0,0,q55,0,0],[0,0,0,0,0,q66,0],[0,0,0,0,0,0,q77]])
	else:
		Q=Matrix([[q11,q12,q13,q14,q15,q16,q17],[q21,q22,q23,q24,q25,q26,q27],[q31,q32,q33,q34,q35,q36,q37],[q41,q42,q43,q44,q45,q46,q47],[q51,q52,q53,q54,q55,q56,q57],[q61,q62,q63,q64,q65,q66,q67],[q71,q72,q73,q74,q75,q76,q77]])

	P=Matrix([[p11,p12,p13,p14,p15,p16,p17],[p21,p22,p23,p24,p25,p26,p27],[p31,p32,p33,p34,p35,p36,p37],[p41,p42,p43,p44,p45,p46,p47],[p51,p52,p53,p54,p55,p56,p57],[p61,p62,p63,p64,p65,p66,p67],[p71,p72,p73,p74,p75,p76,p77]])
	G=Matrix([[g11,g12,g13,g14,g15,g16],[g21,g22,g23,g24,g25,g26],[g31,g32,g33,g34,g35,g36],[g41,g42,g43,g44,g45,g46],[g51,g52,g53,g54,g55,g56],[g61,g62,g63,g64,g65,g66],[g71,g72,g73,g74,g75,g76]])
	# G=Matrix([[g11,0,0,0],[0,g22,0,0],[0,0,g33,0],[0,0,0,g44]])
	H=Matrix([[0,-g,0,0,0,0,0],[g,0,0,0,0,0,0],[0,0,0,0,0,0,1],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0],[0,0,0,0,1,0,0]]) # Measurement matrix to correlate state to observation to find innovation
	if r_sparse:
		R=Matrix([[r11,0,0,0,0,0],[0,r22,0,0,0,0],[0,0,r33,0,0,0],[0,0,0,r44,0,0],[0,0,0,0,r55,0],[0,0,0,0,0,r66]])
		print("Generating equations assuming sparse measurement noise matrix")
	else:
		R=Matrix([[r11,r12,r13,r14,r15,r16],[r21,r22,r23,r24,r25,r26],[r31,r32,r33,r34,r35,r36],[r41,r42,r43,r44,r45,r46],[r51,r52,r53,r54,r55,r56],[r61,r62,r63,r64,r65,r66]])
	
	z=Matrix([[ax],[ay],[az-g],[gyro_roll],[gyro_pitch],[h_lidar]]) # Observation vector
	x=Matrix([[roll],[pitch],[roll_rate],[pitch_rate],[h],[v],[az_dyn]]) # State vector

	expr=[]
	expr.append(A*x)
	expr.append(A*P*A.T + Q)
	expr.append(P*H.T*(H*P*H.T + R).inverse_ADJ())
	expr.append(x + G*(z-H*x))
	expr.append((eye(7) - G*H)*P)

	[(c_name, c_code), (h_name, c_header)] = codegen((('predict_state', expr[0]),
														('predict_cov', expr[1]),
														('update_gain', expr[2]),
														('update_state', expr[3]),
														('update_cov', expr[4])), "C", 'kalman_7state_autogen',project='Kalman_7state_Core')

	src=open(c_name, "w+")
	src.write(c_code)	
	src.close()

	inc=open(h_name, "w+")
	inc.write(c_header)
	inc.close()

if __name__ == '__main__':
	main()