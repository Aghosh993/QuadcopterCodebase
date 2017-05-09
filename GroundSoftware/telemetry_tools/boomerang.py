#!/usr/bin/python2
# boomerang.py
# This script forwards serial port telemetry over UDP in a broadcast fashion
# (c) 2016, Abhimanyu Ghosh
# Based heavily on https://gist.github.com/Lothiraldan/3951784

import os
import serial
import struct
import argparse
from socket import *

class UDP_Boomerang:
	def __init__(self, udp_port, serport, serbaud):
		self.udp_port = udp_port
		self.serport = serial.Serial(serport, serbaud, timeout=None)
		
		self.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
		self.sock.bind(("0.0.0.0", 0))
		self.sock.setsockopt(IPPROTO_IP, IP_MULTICAST_TTL, 255)
		self.sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

	def send_packet(self, data):
		# print("Sending packet")
		# MCAST_ADDR = "237.252.249.227"
		MCAST_ADDR_ETH = "237.252.249.228"
		# self.sock.sendto(data, (MCAST_ADDR, self.udp_port))
		self.sock.sendto(data, (MCAST_ADDR_ETH, self.udp_port))

	def redirect_telemetry_message_stream(self):
		redirect_msg = False

		message_buffer = ""
		start = self.serport.read(1)

		computed_chksum = 0 #ord('s');

		if start == 's':
			message_buffer += start

			desc_size_byte = self.serport.read(1) 
			desc_size = ord(desc_size_byte)
			descriptor = self.serport.read(desc_size)

			message_buffer += desc_size_byte
			computed_chksum += desc_size

			message_buffer += descriptor
			for i in range(len(descriptor)):
				computed_chksum += ord(descriptor[i])

			msg_id_byte = self.serport.read(1) 
			msg_id = ord(msg_id_byte)

			message_buffer += msg_id_byte
			computed_chksum += msg_id

			if msg_id != 3 and msg_id != 4: # I.e. not matrix transfers
				data_len_byte = self.serport.read(1) 
				data_len = ord(data_len_byte)

				message_buffer += data_len_byte
				computed_chksum += data_len

				msg_payload = ""

				if msg_id == 0: # Figure out how many bytes per unit in data_len based on message ID
					msg_payload = self.serport.read(data_len)
					chksum = self.serport.read(2)
				if msg_id == 1 or msg_id == 2:
					msg_payload = self.serport.read(data_len*4)
					chksum = self.serport.read(2)

				message_buffer += msg_payload
				for i in range(len(msg_payload)):
					computed_chksum += ord(msg_payload[i])

				message_buffer += chksum

				computed_chksum += 0x75

				if (computed_chksum & 0xFFFF) == ord(chksum[1])<<8 | ord(chksum[0]):
					redirect_msg = True
					# print(repr(computed_chksum)+" "+repr(ord(chksum[1])<<8 | ord(chksum[0])))

			else:
				rows_byte = self.serport.read(1)
				cols_byte = self.serport.read(1)
				msg_payload = self.serport.read(ord(rows_byte)*ord(cols_byte))
				chksum = self.serport.read(2)

				message_buffer += rows_byte
				message_buffer += cols_byte
				message_buffer += msg_payload
				message_buffer += chksum

			if redirect_msg:
				self.send_packet(message_buffer)

def main():
	parser = argparse.ArgumentParser(description='Redirect telemetry from serial port specified to UDP stream')
	parser.add_argument('baud', metavar='baudrate', type=int, nargs=1, help='Set serial port baud rate')
	parser.add_argument('serialport', metavar='serial_port', nargs=1, help='Path to serial port that is source of telemetry')
	parser.add_argument('udp_port', metavar='UDP_Portnum', type=int, nargs=1, help='UDP port to broadcast telemetry over')

	args = parser.parse_args()

	serial_baud_rate = args.baud[0]
	serial_port = args.serialport[0]
	udp_port = args.udp_port[0]

	bouncer = UDP_Boomerang(udp_port, serial_port, serial_baud_rate)
	while True:
		bouncer.redirect_telemetry_message_stream()

if __name__ == '__main__':
	main()