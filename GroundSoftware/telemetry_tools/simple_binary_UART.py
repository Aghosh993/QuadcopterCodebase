import os;
import serial;
import struct;

# Some portions of graphical plotting code adapted from http://matplotlib.org/examples/pylab_examples/polar_demo.html
# and http://jakevdp.github.io/blog/2012/08/18/matplotlib-animation-tutorial/

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

fig = plt.figure()

# Max length and width of plot, in meters:
rate_limits	= 50;
time_window	= 100;

ax = plt.axes(xlim=(0,time_window),ylim=(-1*rate_limits,rate_limits))
line, = ax.plot([], [], '-')

serialPort = serial.Serial();
data_buf = [];
x_vals = np.arange(0,time_window,1); #Fixed X-values

def init_port():

	global serialPort;

	try:
		serialPort = serial.Serial("/dev/ttyACM0", 115200);

	except:
		print("Could not open serial port, please check your privilege!");
		exit();

	for i in range(time_window):
		data_buf.append(0.);

def get_data():

	global serialPort;
	global data_buf;

	for i in range(time_window-1):
		data_buf[i] = data_buf[i+1]; #Shift data back to make space for next element incoming

	serialPort.flush();
	start_byte = serialPort.read(1);

	if start_byte == 's':
		message_id = serialPort.read(1);
		message_len = serialPort.read(1);
		in_bytes = serialPort.read(4);
		checksum = serialPort.read(1);

		checksum_calc = ord(in_bytes[0]) + ord(in_bytes[1]) + ord(in_bytes[2]) + ord(in_bytes[3]) + 0x75;
		if (checksum_calc & 0xFF) != ord(checksum):
			print("Checksum error, message ID: "+repr(message_id)+" received checksum: "+ repr(ord(checksum))+" vs calculated value of: "+repr(checksum_calc & 0xFF));
		else:
			roll_val = struct.unpack('f', in_bytes)[0];
			data_buf[time_window-1] = roll_val; #Fill in the last (latest) element of data buffer
			print repr(roll_val);

def init():

	line.set_data([],[])#x_vals, data_buf)
	return line,

def update_fig(i):
	global x_vals
	global data_buf

	get_data()

	# Refresh the plot with recalculated points:
	line.set_data(x_vals,data_buf)
	return line,

def main():
	# Set up the serial port, do mode switch on LIDAR and initialize global data structures:
	init_port()

	#while True:
	#	get_data();

	# Set up animation:
	myAnimation = animation.FuncAnimation(fig,update_fig,init_func=init,frames=10000,interval=20,blit=True)

	# Render animation. This will also keep calling the update function defined to collect LIDAR data and refresh the plot:
	plt.show()

if __name__ == '__main__':
	main()