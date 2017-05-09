#!/usr/bin/python2
# udp_prism.py
# This script connects to a UDP port and receives telemetry and looks for desired messages with a specific message ID and descriptor
# Messages satisfying the above filter are then graphed
# (c) 2016, Abhimanyu Ghosh
import os
import sys
import serial
import argparse
from socket import *
import select

from struct import *
from time import sleep

import numpy as np
import pyqtgraph as pq
import math

# For WiFi Multicast:
MCAST_IP = "237.252.249.227"

# For Ethernet Multicast:
# MCAST_IP = "237.252.249.228"

class Scope(object):
    def __init__(self, ax, maxt=10, dt=0.02, zoom_step=1.5):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(-1000, 10000)
        self.ax.set_xlim(0, self.maxt)
        self.zoom_step = zoom_step

    # A function to auto-resize the Y-axis in response to the true range of data from the previous "maxt" worth of data points
    def resize_y(self, vert_buffer=0.2):
        # First get the Y-axis limits at present:
        vert_limits = self.ax.get_ylim()
        ymin = vert_limits[0]
        ymax = vert_limits[1]

        # Counters to track data max and min:
        ydata_max = 0.0
        ydata_min = 0.0

        # Look through the data to find true max and min:
        for i in range(len(self.ydata)):
            if self.ydata[i] > ydata_max:
                ydata_max = self.ydata[i]
            if self.ydata[i] < ydata_min:
                ydata_min = self.ydata[i]

        # Update desired Y-axis limits, and add in a buffer space so points at the edge are more visible:
        ymax = ydata_max + vert_buffer
        ymin = ydata_min - vert_buffer

        # Propagate these changes to the plot:
        self.ax.set_ylim(ymin, ymax)

    def update(self, y):
    	if y is not None:
	        # Get the last "x-axis" point in the array for that axis:
	        lastt = self.tdata[-1]
	        # If we're at the end of a horizontal period (i.e. "maxt" number of points collected)
	        # we reset the array elements, recompute Y-axis limits and shift the X-axis forward another "maxt" amount:
	        if lastt > self.tdata[0] + self.maxt:
	            self.tdata = [self.tdata[-1]]
	            self.resize_y()
	            self.ydata = [self.ydata[-1]]
	            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
	            self.ax.figure.canvas.draw()

	        # We keep putting new data into the Y-axis buffer and adding to the X-axis:
	        # t = self.tdata[-1] + self.dt
	        self.tdata.append(y[0])
	        self.ydata.append(y[1])
	        self.line.set_data(self.tdata, self.ydata)
    	return self.line,

    # A callback for Matplotlib to go to when the user scrolls in/out while having cursor over the plot area:
    def time_zoom_handler(self, event):
        # A simple exponential zoom. event.step determines if the user is scrolling in/out 
        # and thus the direction of our zooming action:
        zoom_multiplier = 1.0
        zoom_multiplier *= math.exp(event.step*self.zoom_step)

        self.maxt *= zoom_multiplier

        # If the user is zooming out, we want to trigger a re-draw so we're not waiting forever 
        # for a whole new set of data... just "stretch" the existing data to fit the new range
        # and resize the X-axis appropriately:

        if self.tdata[-1] < self.tdata[0] + self.maxt:
            self.line.set_data(self.tdata, self.ydata)
            self.ax.set_xlim(self.tdata[0], self.tdata[-1])
            self.ax.figure.canvas.draw()
            self.tdata = [self.tdata[-1]]
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)

class telem_processor:
	def __init__(self, udp_port, msg_id, desc, pos_offset):
		self.udp_port = udp_port

		self.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
		self.sock.setsockopt(SOL_SOCKET, SO_REUSEPORT, 1)
		# Bind on all interfaces:
		self.sock.bind(("0.0.0.0", self.udp_port))

		self.sock.setsockopt(IPPROTO_IP, IP_MULTICAST_TTL, 255)

		self.sock.setsockopt(IPPROTO_IP, IP_ADD_MEMBERSHIP, inet_aton(MCAST_IP) + inet_aton("0.0.0.0"))
		
		self.msg_id_filter = msg_id
		self.descriptor_filter = desc
		self.pos = pos_offset

	def get_n_floats_message(self):
		res = select.select([self.sock],[],[])
		# msg=[]
		# while len(msg)==0:
		msg = res[0][0].recvfrom(1024)
		msg_data = msg[0]

		if msg_data[0] == 's':
			desc_size = ord(msg_data[1])
			descriptor = ""
			for i in range(desc_size):
				descriptor += msg_data[i+2]

				if descriptor == self.descriptor_filter:
					msg_id = ord(msg_data[desc_size+2])
					if msg_id == self.msg_id_filter:
						data_len = ord(msg_data[desc_size+3])				
						float_list = []
						for i in range(data_len):
							msg_payload_segment = ""
							for j in range(4):
								msg_payload_segment += msg_data[desc_size+4+(i*4)+j]
							float_list.append(unpack('<f', msg_payload_segment)[0])	

						return float_list
		
	def get_n_ints_message(self):
		res = select.select([self.sock],[],[])
		# msg=[]
		# while len(msg)==0:
		msg = res[0][0].recvfrom(1024)
		msg_data = msg[0]

		if msg_data[0] == 's':
			desc_size = ord(msg_data[1])
			descriptor = ""
			for i in range(desc_size):
				descriptor += msg_data[i+2]

				if descriptor == self.descriptor_filter:
					msg_id = ord(msg_data[desc_size+2])
					if msg_id == self.msg_id_filter:
						data_len = ord(msg_data[desc_size+3])				
						int_list = []
						for i in range(data_len):
							msg_payload_segment = ""
							for j in range(4):
								msg_payload_segment += msg_data[desc_size+4+(i*4)+j]
							int_list.append(unpack('<i', msg_payload_segment)[0])	

						fake_float_list = []
						fake_float_list.append(float(int_list[0])/1000)
						fake_float_list.append(float(int_list[1]))

						print(repr(fake_float_list[0])+", "+repr(fake_float_list[1]) + ", " + repr(int_list[1]))

						return fake_float_list

	def push_data_to_buffer(self):
		if self.msg_id_filter == 1:
				yield self.get_n_floats_message()
				# return data
		if self.msg_id_filter == 2:
				yield self.get_n_ints_message()
				# return data

def main():

	parser = argparse.ArgumentParser(description='Display telemetry item with specified descriptor string and message id')
	parser.add_argument('port', metavar='UDP_port', type=int, nargs=1, help='UDP Port to listen for broadcast messages on')
	parser.add_argument('msg_id', metavar='messageID', nargs=1, type=int, help='Message ID of the telemetry item to be viewed. 1 for n-floats and 2 for n-ints')
	parser.add_argument('pos', metavar='pos_identifier', nargs=1, type=int, help='Item within telemetry item to graph. Range from 0 to size(message)-1')
	parser.add_argument('desc_string', metavar='descriptor_string', nargs=1, help='Descriptor string of the telemetry message to be viewed')

	args = parser.parse_args()

	udp_port = args.port[0]
	msgID = args.msg_id[0]
	desc = args.desc_string[0]
	position_to_display = args.pos[0]

	print("Opening client on UDP port "+repr(udp_port))

	s = telem_processor(udp_port, msgID, desc, position_to_display)

	fig, ax = plt.subplots()
	scope = Scope(ax, zoom_step=0.2)
	f2 = ax.get_figure()

	ani = animation.FuncAnimation(fig, scope.update, s.push_data_to_buffer, interval=1, blit=True)

	f2.canvas.mpl_connect('scroll_event', scope.time_zoom_handler)

	plot_title = "Msg: " + desc + " MSG ID:" + repr(msgID)
	plt.title(plot_title)

	plt.show()

if __name__ == '__main__':
	main()