#!/usr/bin/python3
import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
import argparse

def main():
	parser = argparse.ArgumentParser(description='Display a CSV file')
	parser.add_argument('filename', metavar='filename', nargs=1, help='CSV file to open')

	args = parser.parse_args()

	fn = args.filename[0]

	input_data = np.loadtxt(fn, delimiter=', ')

	xdata = input_data[:,0]
	ydata = input_data[:,1]

	fig = plt.figure()
	plot1 = fig.add_subplot(111)
	plot1.plot(xdata,ydata)
	plt.show()

if __name__ == '__main__':
	main()