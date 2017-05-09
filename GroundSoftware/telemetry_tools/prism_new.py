#!/usr/bin/python2
# prism_new.py
# This script directly connects to a serial port and receives telemetry and looks for desired messages with a specific message ID and descriptor
# Messages satisfying the above filter are then displayed on the CLI
# (c) 2017, Abhimanyu Ghosh
import os
import sys
import serial
import argparse

from struct import *
from time import sleep

class telemetry_prism:
	def __init__(self, port, baud):
		self.serport = serial.Serial(port, baud)
		self.buffer = list()

	def seek_to_start(self, buf):
		for i in range(len(buf)):
			if buf[i] == 's':
				return i
		return -1

	def get_data(self):
		if self.serport.inWaiting() > 0:
			buf = self.serport.read(self.serport.inWaiting())
			for ch in buf:
				self.buffer.append(ch)

	def process_data(self):
		if len(self.buffer) > 0:
			start_pos = seek_to_start(self.buffer)
			if start_pos != -1:
				for i in range(start_pos):
					c = self.buffer.pop(0)
				c = self.buffer.pop(0)
				while(len)
				desc_size = self.buffer.pop(0)

	def display_filtered_message(self, msg_id, desc_string_filter):
		if msg_id == 0:
			while True:
				self.get_string_message(desc_string_filter)
		if msg_id == 1:
			while True:
				self.get_n_floats_message(desc_string_filter)
		if msg_id == 2:
			while True:
				self.get_n_ints_message(desc_string_filter)
		if msg_id == 3:
			while True:
				self.get_m_n_float_matrix_message(desc_string_filter)
		if msg_id == 4:
			while True:
				self.get_m_n_int_matrix_message(desc_string_filter)

def main():

	parser = argparse.ArgumentParser(description='Display telemetry item with specified descriptor string and message id')
	parser.add_argument('baud', metavar='baudrate', type=int, nargs=1, help='Set serial port baud rate')
	parser.add_argument('port', metavar='serial_port', nargs=1, help='Path to serial port that is source of telemetry')
	parser.add_argument('msg_id', metavar='messageID', nargs=1, type=int, help='Message ID of the telemetry item to be viewed')
	parser.add_argument('desc_string', metavar='descriptor_string', nargs=1, help='Descriptor string of the telemetry message to be viewed')

	args = parser.parse_args()

	baud_rate = args.baud[0]
	serial_port = args.port[0]
	msgID = args.msg_id[0]
	desc = args.desc_string[0]

	s = telemetry_prism(serial_port, baud_rate)
	s.display_filtered_message(msgID, desc)

if __name__ == '__main__':
	main()