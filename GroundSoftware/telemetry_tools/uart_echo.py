import os
import sys
import serial
from time import sleep

def main():
	serialport = serial.Serial(sys.argv[1], 115200)

	while True:
		serialport.write(b'Hello')
		reply = serialport.read(5)
		if reply == "Hello":
			print("Pass")
		else:
			print("Fail!!")
		
		# sleep(0.001)

if __name__ == '__main__':
	main()