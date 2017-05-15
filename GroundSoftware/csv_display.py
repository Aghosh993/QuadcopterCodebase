#!/usr/bin/python3
import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
import argparse

message_set = "sf11_bno055 v_z ahrs_rp yaw_height flow bno055_att esc_cmds"

def plot_file(file):
	fig = plt.figure()

	input_data = np.loadtxt(file, delimiter=', ')

	col = input_data.shape[1]
	xdata = input_data[:,0]
	subplot_mask = 100+(col-1)*10

	ydata = []
	plots = []

	for i in range(col-1):
		ydata.append(input_data[:,i+1])
		plots.append(fig.add_subplot(subplot_mask+i+1))
		plots[i].plot(xdata,ydata[i])
	plt.show()

def plot_fileset(files):
	fig = []
	j=0

	for file in files:
		input_data = np.loadtxt(file, delimiter=', ')

		if input_data.size > 2:

			fig.append(plt.figure())
			fig[j].suptitle(file)
			col = input_data.shape[1]
			xdata = input_data[:,0]
			subplot_mask = 100+(col-1)*10

			ydata = []
			plots = []

			for i in range(col-1):
				ydata.append(input_data[:,i+1])
				plots.append(fig[j].add_subplot(subplot_mask+i+1))
				plots[i].plot(xdata,ydata[i])
			j = j+1	
	
	plt.show()

def main():
	parser = argparse.ArgumentParser(description='Display a CSV file')
	parser.add_argument('-t', '--timestamp', metavar='timestamp', nargs=1, help='UTC timestamp of experimental fileset to plot')
	parser.add_argument('-d', '--log_dir', metavar='log_dir', nargs=1, help='Path to directory containing experimental fileset')
	parser.add_argument('-f', '--filename', metavar='filename', nargs=1, help='CSV file to open')

	args = parser.parse_args()

	if args.filename:
		fn = args.filename[0]
		plot_file(fn)
	else:
		print("Attempting to plot full experimental run from time ")

		if args.timestamp:
			ts = args.timestamp[0]
			msgs = message_set.split()

			if args.log_dir:
				logdir = args.log_dir[0]
			else:
				logdir = "../logs"

			file_set = []
			for msg in msgs:
				file_name = logdir+"/"+msg+"_"+ts+".csv"
				file_set.append(file_name)
			plot_fileset(file_set)

if __name__ == '__main__':
	main()