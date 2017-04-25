#!/usr/bin/python3
import os
import sys
import argparse
from struct import *
import can

def main():
	parser = argparse.ArgumentParser(description='Display CAN bus telemetry items')
	parser.add_argument('sensor_id', metavar='sensor', nargs=1, help='Sensor ID to get data for. Options: sf11_bno055, flow')

	args = parser.parse_args()

	sensor = args.sensor_id[0]
	if sensor != "sf11_bno055" and sensor != "flow":
		print("Usage: ./sensor_tester.py [SENSOR]")
		print("[SENSOR]=sf11_bno55 or flow")
		exit()

	print("Welcome to the CAN tester")

	can_iface = "can0" # Change this to the CAN interface identifier shown by ifconfig

	if sensor == "sf11_bno055":
		arbID = 1
		print("Acquiring height and heading sensor data now...")
	if sensor == "flow":
		arbID = 2
		print("Acquiring X and Y flow sensor data now...")

	print("Attempting to open SocketCAN interface on can0...")
	bus = can.interface.Bus(can_iface, bustype='socketcan_native')

	while True:
		msg = bus.recv()
		if(msg.arbitration_id == arbID and msg.is_extended_id==False):
			height = unpack('<f', msg.data[0:4])[0]
			print(height)
			# for i in range(0, msg.dlc):
				# sys.stdout.write(chr(msg.data[i]))
				# sys.stdout.flush()


if __name__ == '__main__':
	main()