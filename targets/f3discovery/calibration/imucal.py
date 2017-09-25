#!/usr/bin/python3
from sympy import *
from sympy.utilities.codegen import codegen
import argparse
import numpy as np
import matplotlib as mp
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from mpl_toolkits.mplot3d import Axes3D

# A simple script that invokes Sympy to implement a Newton-Gauss least-squares algorithm to determine coefficients
# for a given set of IMU data. This script may be applied to either Accelerometer or Magnetometer data, with some 
# minor tweaking of the convergence criteria
# 
# Distributed under the BSD 3-clause license, as noted below:
# 
# (c) 2017, Abhimanyu Ghosh
# 
# Redistribution and use in source and binary forms, with or without modification, are permitted provided 
# that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions 
# 	 and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
#    and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
#    promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 

def main():
	parser = argparse.ArgumentParser(description='Generate IMU calibration coefficients from CSV log file')
	parser.add_argument('-f', '--file', metavar='file', nargs=1, help='Path to CSV file to use as input')
	parser.add_argument('-m', '--magnitude', metavar='magnitude', nargs=1, help='Target magnitude of IMU readings. Ex: for accelerometer set this to 1 (g) or 9.8 (m*s^-2)')

	args = parser.parse_args()

	if args.file:
		fn = args.file[0]
	else:
		fn = "log.csv"
		print("Using default file "+repr(fn))

	try:
		input_data = np.loadtxt(fn, delimiter=', ')
	except IOError:
		print("ERROR opening file, aborting!!")
		exit(-1)

	if args.magnitude:
		mag = args.magnitude[0]
	else:
		print("Fitting data to default unit sphere as magnitude arg is not provided...")
		mag = 1.

	rows = input_data.shape[0]

	ax,ay,az = symbols('ax,ay,az')
	sx,sy,sz,bx,by,bz = symbols('sx,sy,sz,bx,by,bz')
	r = symbols('r')

	r = sx**2*(ax-bx)**2 + sy**2*(ay-by)**2 + sz**2*(az-bz)**2 - 1.0**2
	rsq = (sx**2*(ax-bx)**2 + sy**2*(ay-by)**2 + sz**2*(az-bz)**2 - 1.0**2)**2
	params = Matrix([sx,sy,sz,bx,by,bz])

	convergence_threshold = 5
	r_sq_eval = lambdify((ax,ay,az,sx,sy,sz,bx,by,bz), rsq)
	
	sx_calc=1.
	sy_calc=1.
	sz_calc=1.
	bx_calc=0.
	by_calc=0.
	bz_calc=0.

	residualSetJacobian = []
	residualSet = []
	for row in input_data:
		r.subs(ax,row[0])
		r.subs(ay,row[1])
		r.subs(az,row[2])
		r_eval = lambdify((ax,ay,az),r)
		residualSetJacobian.append(Matrix([r_eval(row[0], row[1], row[2])]).jacobian(params))
		residualSet.append(Matrix([r_eval(row[0], row[1], row[2])]))
	
	# Matrix of the array of expressions containing the partly-evaluated Jacobians:
	sym_jacobian = Matrix(residualSetJacobian)

	# Matrix of the array of expressions containing the residuals:
	sym_residuals = Matrix(residualSet)

	# Evaluable lambda that allows evaluation of above Jacobian as a function of the tunable parameters
	# i.e. the 3 scale factors and the 3 biases
	evalJacobian = lambdify((sx,sy,sz,bx,by,bz),sym_jacobian)

	# Evaluable lambda that allows evaluation of above Residuals as a function of the tunable parameters
	# i.e. the 3 scale factors and the 3 biases
	evalResiduals = lambdify((sx,sy,sz,bx,by,bz),sym_residuals)

	while True:
		err_sq = 0.
		for row in input_data:
			err_sq += r_sq_eval(row[0],row[1],row[2],sx_calc,sy_calc,sz_calc,bx_calc,by_calc,bz_calc)
		if err_sq < convergence_threshold:
			fig = plt.figure()
			plt.hold(True)
			rawSamples = input_data
			correctedSamples = np.copy(rawSamples)
			for sample in correctedSamples:
				# correctedSamplesX.append(sx_calc*(sample[0]-bx_calc))
				# correctedSamplesY.append(sy_calc*(sample[1]-by_calc))
				# correctedSamplesZ.append(sz_calc*(sample[2]-bz_calc))
				sample[0] = sx_calc*(sample[0]-bx_calc)
				sample[1] = sy_calc*(sample[1]-by_calc)
				sample[2] = sz_calc*(sample[2]-bz_calc)

			pi = np.pi
			sin = np.sin
			cos = np.cos
			phi,theta = np.mgrid[-1.0*pi:pi:20j, -0.5*pi:0.5*pi:10j]

			x = float(sx_calc)*(cos(theta)*cos(phi)) - float(sx_calc*bx_calc)
			y = float(sy_calc)*(cos(theta)*sin(phi)) - float(sy_calc*by_calc)
			z = float(sz_calc)*(sin(theta)) -float(sz_calc*bz_calc)
			# print(z)

			# subplot1 = fig.add_subplot(121, projection='3d')
			subplot2 = fig.add_subplot(111, projection='3d')
			subplot2.plot_surface(x, y, z,  rstride=1, cstride=1, color='c', alpha=0.2, linewidth=0, cmap=cm.hot)
			# subplot2.set_xlim([-1e-3,1e-3])
			# subplot2.set_ylim([-1e-3,1e-3])
			# subplot2.set_zlim([-1e-3,1e-3])
			# subplot1.scatter(rawSamples[:,0], rawSamples[:,1], rawSamples[:,2])
			# print(np.asarray(correctedSamplesX))
			subplot2.scatter(correctedSamples[:,0], correctedSamples[:,1], correctedSamples[:,2], color="k", s=20)
			plt.show()
			print("float sx="+repr(sx_calc)+";")
			print("float sy="+repr(sy_calc)+";")
			print("float sz="+repr(sz_calc)+";")
			print("float bx="+repr(bx_calc)+";")
			print("float by="+repr(by_calc)+";")
			print("float bz="+repr(bz_calc)+";")
			break
		else:
			currentJacobian = evalJacobian(sx_calc,sy_calc,sz_calc,bx_calc,by_calc,bz_calc)
			currentResiduals = evalResiduals(sx_calc,sy_calc,sz_calc,bx_calc,by_calc,bz_calc)
			adjustment = Matrix(Matrix((Matrix(currentJacobian.T).multiply(currentJacobian)).inv()).multiply(currentJacobian.T)).multiply(currentResiduals)
			
			sx_calc = sx_calc - adjustment[0]
			sy_calc = sy_calc - adjustment[1]
			sz_calc = sz_calc - adjustment[2]
			bx_calc = bx_calc - adjustment[3]
			by_calc = by_calc - adjustment[4]
			bz_calc = bz_calc - adjustment[5]

if __name__ == '__main__':
	main()