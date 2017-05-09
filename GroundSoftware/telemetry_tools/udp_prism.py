#!/usr/bin/python2
# udp_prism.py
# This script connects to a UDP port and receives telemetry and looks for desired messages with a specific message ID and descriptor
# Messages satisfying the above filter are then displayed on the CLI
# (c) 2016, Abhimanyu Ghosh
import os
import sys
import serial
import argparse
from socket import *
import select

from struct import *
from time import sleep

# For WiFi Multicast:
MCAST_IP = "237.252.249.227"

# For Ethernet Multicast:
#MCAST_IP = "237.252.249.228"

class telemetry_prism:
	def __init__(self, udp_port, msg_id, desc):
		self.udp_port = udp_port

		self.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
		self.sock.setsockopt(SOL_SOCKET, SO_REUSEPORT, 1)
		# Bind on all interfaces:
		self.sock.bind(("0.0.0.0", self.udp_port))

		# Join multicast group:
		self.sock.setsockopt(IPPROTO_IP, IP_MULTICAST_TTL, 255)

		self.sock.setsockopt(IPPROTO_IP, IP_ADD_MEMBERSHIP, inet_aton(MCAST_IP) + inet_aton("0.0.0.0"))

		self.msg_id_filter = msg_id
		self.descriptor_filter = desc

	def get_string_message(self):
		res = select.select([self.sock],[],[])
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
						data = ""
						for i in range(data_len):
							data += msg_data[desc_size+4+i]
						print(data)

	def get_n_floats_message(self):
		res = select.select([self.sock],[],[])
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
						print(float_list)
		
	def get_n_ints_message(self):
		res = select.select([self.sock],[],[])
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
						print(int_list)

	def display_filtered_message(self):
		if self.msg_id_filter == 0:
			while True:
				self.get_string_message()
		if self.msg_id_filter == 1:
			while True:
				self.get_n_floats_message()
		if self.msg_id_filter == 2:
			while True:
				self.get_n_ints_message()

def main():

	parser = argparse.ArgumentParser(description='Display telemetry item with specified descriptor string and message id')
	parser.add_argument('port', metavar='UDP_port', type=int, nargs=1, help='UDP Port to listen for broadcast messages on')
	parser.add_argument('msg_id', metavar='messageID', nargs=1, type=int, help='Message ID of the telemetry item to be viewed')
	parser.add_argument('desc_string', metavar='descriptor_string', nargs=1, help='Descriptor string of the telemetry message to be viewed')

	args = parser.parse_args()

	udp_port = args.port[0]
	msgID = args.msg_id[0]
	desc = args.desc_string[0]

	print("Opening client on UDP port "+repr(udp_port))

	s = telemetry_prism(udp_port, msgID, desc)
	s.display_filtered_message()

if __name__ == '__main__':
	main()
