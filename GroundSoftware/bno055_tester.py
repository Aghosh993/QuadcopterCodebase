#!/usr/bin/python3
import sys
import serial
import time
from struct import *
import argparse

class BNO055:
	def __init__(self, port):
		self.port = port

	def write_single_reg(self, reg, data):
		cmd = b"\xAA\x00"
		cmd += chr(reg).encode()
		cmd += b"\x01"
		cmd += chr(data).encode()
		self.port.write(cmd)
		res = self.port.read(2)

	def request_read_reg(self, reg, read_len):
		cmd = b"\xAA\x01"
		cmd += chr(reg).encode()
		cmd += chr(read_len).encode()
		self.port.write(cmd)

	def get_fused_data(self):
		self.request_read_reg(0x14, 20)
		header = ord(self.port.read(1))
		if header == 0xBB:
			reply_len = ord(self.port.read(1))
			buf = self.port.read(reply_len)
			data = unpack('<hhhhhhhhhh', buf)
			print("Heading: "+repr(data[3]/16)+" Roll: "+repr(data[4]/16)+" Pitch: "+repr(data[5]/16))

		else:
			if header == 0xEE:
				err = self.port.read(1)

def main():
	parser = argparse.ArgumentParser(description='Show fused data from a BNO055 sensor module in UART interface mode')
	s = serial.Serial("/dev/ttyUSB3", 115200)
	b = BNO055(s)
	b.write_single_reg(0x3d, 0x00)
	time.sleep(1)
	b.write_single_reg(0x3d, 0x0C)
	time.sleep(1)
	while True:
		b.get_fused_data()
		time.sleep(0.02)

if __name__ == '__main__':
	main()