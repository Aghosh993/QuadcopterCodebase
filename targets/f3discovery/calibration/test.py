#!/usr/bin/python3
from sympy import *
from sympy.utilities.codegen import codegen
import argparse
import numpy as np
import matplotlib as mp
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from mpl_toolkits.mplot3d import Axes3D

def main():
	pi = np.pi
	sin = np.sin
	cos = np.cos
	# theta = np.linspace(-0.5*pi, 0.5*pi, 100)
	# phi = np.linspace(-1.0*pi, 1.0*pi, 100)
	phi,theta = np.mgrid[-1.0*pi:pi:10j, -0.5*pi:0.5*pi:10j]
	x = 0.1*(cos(theta)*cos(phi))
	y = 0.2*(cos(theta)*sin(phi)) 
	z = 0.2*(sin(theta))

	# r = 1
	# pi = np.pi
	# cos = np.cos
	# sin = np.sin
	# phi, theta = np.mgrid[-pi:pi:100j, -pi/2.0:pi/2.0:100j]
	# x = r*sin(phi)*cos(theta)
	# y = r*sin(phi)*sin(theta)
	# z = r*cos(phi)

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot_surface(x, y, z,  rstride=1, cstride=1, color='c', alpha=0.6, linewidth=0)
	# subplot2.set_aspect("equal")
	ax.set_xlim([-1,1])
	ax.set_ylim([-1,1])
	ax.set_zlim([-1,1])
	plt.show()

if __name__ == '__main__':
	main()