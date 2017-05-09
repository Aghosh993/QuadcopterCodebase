#!/usr/bin/python2
# prism.py
# This script directly connects to a serial port and receives telemetry and looks for desired messages with a specific message ID and descriptor
# Messages satisfying the above filter are then displayed on the CLI
# (c) 2016, Abhimanyu Ghosh
import os
import sys
import serial
import argparse

from struct import *
from time import sleep

class telemetry_prism:
	def __init__(self, port, baud):
		self.serport = serial.Serial(port, baud)

	def get_string_message(self, desc):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			descriptor = self.serport.read(desc_size)

			if descriptor == desc:
				msg_id = ord(self.serport.read(1))
				if msg_id == 0:
					data_len = ord(self.serport.read(1))
					msg_payload = self.serport.read(data_len)
					chksum = self.serport.read(2)

					sys.stdout.write(msg_payload)

	def get_n_floats_message(self, desc):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			descriptor = self.serport.read(desc_size)

			if descriptor == desc:
				msg_id = ord(self.serport.read(1))
				if msg_id == 1:
					data_len = ord(self.serport.read(1))				
					float_list = []
					for i in range(data_len):
						msg_payload_segment = self.serport.read(4)	
						float_list.append(unpack('<f', msg_payload_segment)[0])

					chksum = self.serport.read(2)

					print(float_list)				

	def get_n_ints_message(self, desc):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			# print(desc_size)
			descriptor = self.serport.read(desc_size)

			if descriptor == desc:
				msg_id = ord(self.serport.read(1))
				if msg_id == 2:
					data_len = ord(self.serport.read(1))				
					int_list = []
					for i in range(data_len):
						msg_payload_segment = self.serport.read(4)	
						int_list.append(unpack('<i', msg_payload_segment)[0])

					chksum = self.serport.read(2)
					
					print(int_list)

	# def get_n_ints_message(self, desc):
	# 	st = self.serport.read(1)
	# 	if st == 's':
	# 		desc_size = ord(self.serport.read(1))
	# 		# print(desc_size)
	# 		descriptor = self.serport.read(desc_size)
	# 		msg_id = ord(self.serport.read(1))
	# 		data_len = ord(self.serport.read(1))
	# 		# print(data_len)			
	# 		int_list = []
	# 		for i in range(data_len):
	# 			msg_payload_segment = self.serport.read(4)	
	# 			int_list.append(unpack('<i', msg_payload_segment)[0])

	# 		chksum = self.serport.read(2)
			
	# 		# if descriptor == desc and msg_id == 2:
	# 		print(int_list)

	def get_m_n_float_matrix_message(self, desc):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			descriptor = self.serport.read(desc_size)

			if descriptor == desc:
				msg_id = ord(self.serport.read(1))
				if msg_id == 3:
					rows = ord(self.serport.read(1))
					print("ROWS: "+repr(rows))
					cols = ord(self.serport.read(1))
					print("COLS: "+repr(cols))
					
					mat = []
					
					for i in range(rows):
						mat_col = []
						for j in range(cols):
							msg_payload_segment = self.serport.read(4)	
							mat_col.append(unpack('<f', msg_payload_segment)[0])
						mat.append(mat_col)
						sys.stdout.write(repr(mat_col))
						sys.stdout.write("\n")

					chksum = self.serport.read(2)
					sys.stdout.write("\n")

	def get_m_n_int_matrix_message(self, desc):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			descriptor = self.serport.read(desc_size)

			if descriptor == desc:
				msg_id = ord(self.serport.read(1))
				if msg_id == 4:
					rows = ord(self.serport.read(1))
					print("ROWS: "+repr(rows))
					cols = ord(self.serport.read(1))
					print("COLS: "+repr(cols))
					
					mat = []
					
					for i in range(rows):
						mat_col = []
						for j in range(cols):
							msg_payload_segment = self.serport.read(4)	
							mat_col.append(unpack('<i', msg_payload_segment)[0])
						mat.append(mat_col)
						sys.stdout.write(repr(mat_col))
						sys.stdout.write("\n")

					chksum = self.serport.read(2)
					sys.stdout.write("\n")

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