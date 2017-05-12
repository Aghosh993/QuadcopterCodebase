#!/usr/bin/python3
import os
import sys
import argparse
from struct import *
import can
import time

# An application to listen for data on a Socket CAN interface and log/display data items of interest
# (c) Abhimanyu Ghosh, 2017

def main():
	parser = argparse.ArgumentParser(description='Display and log CAN bus telemetry items')
	parser.add_argument('can_iface', metavar='can_iface', nargs=1, help='Interface identifier of the CAN interface to use')
	parser.add_argument('sensor_id', metavar='sensor', nargs=1, help='Sensor ID to get data for. Options: sf11_bno055, flow, ahrs_rp, yaw_height, v_z')
	parser.add_argument('logfile', metavar='log_file', nargs=1, help='File to log the raw data to')
	parser.add_argument('-so', '--show_output', help='Print output for debugging', action="store_true")

	args = parser.parse_args()

	can_iface = args.can_iface[0]
	sensor = args.sensor_id[0]
	filename = args.logfile[0]

	verboseMode = False
	if args.show_output:
		verboseMode = True

	try:
		fp = open(filename, "w+")
	except IOError:
		print("Failed to open log file!!")
		exit()

	print("Welcome to the CAN tester")

	can_iface = "can0" # Change this to the CAN interface identifier shown by ifconfig
	print("Attempting to open SocketCAN interface on can0...")
	bus = can.interface.Bus(can_iface, bustype='socketcan_native')
	t0 = time.time() # Initial time of start
	firstPacket = True

	if sensor == "sf11_bno055":
		print("Acquiring height and heading sensor data now...")
		arbID = 1
		while True:
			msg = bus.recv()
			if(msg.arbitration_id == arbID and msg.is_extended_id==False):
				height_heading_msg = unpack('<ff', msg.data[0:8])
				if firstPacket:
					t0 = time.time()
					firstPacket = False
				fp.write("%0.3f"%(time.time()-t0)+", %0.2f"%height_heading_msg[0]+", %0.1f\n"%height_heading_msg[1])
				if verboseMode:
					print("Height: %0.2f"%height_heading_msg[0]+", Heading: %0.1f"%height_heading_msg[1])

	elif sensor == "flow":
		print("Acquiring X and Y flow sensor data now...")
		arbID = 2
		while True:
			msg = bus.recv()
			if(msg.arbitration_id == arbID and msg.is_extended_id==False):
				flow_msg = unpack('<ff', msg.data[0:8])
				if firstPacket:
					t0 = time.time()
					firstPacket = False
				fp.write("%0.3f"%(time.time()-t0)+", %0.2f"%flow_msg[0]+", %0.2f\n"%flow_msg[1])
				if verboseMode:
					print("X: %0.2f"%flow_msg[0]+", Y: %0.2f"%flow_msg[1])
				
	elif sensor == "ahrs_rp":
		print("Acquiring AHRS roll and pitch data now...")
		arbID = 3
		while True:
			msg = bus.recv()
			if(msg.arbitration_id == arbID and msg.is_extended_id==False):
				ahrs_rp_msg = unpack('<ff', msg.data[0:8])
				if firstPacket:
					t0 = time.time()
					firstPacket = False
				fp.write("%0.3f"%(time.time()-t0)+", %0.2f"%ahrs_rp_msg[0]+", %0.2f\n"%ahrs_rp_msg[1])
				if verboseMode:
					print("Roll: %0.2f"%ahrs_rp_msg[0]+", Pitch: %0.2f"%ahrs_rp_msg[1])
				
	elif sensor == "yaw_height":
		print("Acquiring yaw sensor and height estimator data now...")
		arbID = 4
		while True:
			msg = bus.recv()
			if(msg.arbitration_id == arbID and msg.is_extended_id==False):
				yaw_height_msg = unpack('<ff', msg.data[0:8])
				if firstPacket:
					t0 = time.time()
					firstPacket = False
				fp.write("%0.3f"%(time.time()-t0)+", %0.2f"%yaw_height_msg[0]+", %0.2f\n"%yaw_height_msg[1])
				if verboseMode:
					print("Yaw: %0.2f"%yaw_height_msg[0]+", Height_Est: %0.2f"%yaw_height_msg[1])
				
	elif sensor == "v_z":
		print("Acquiring vertical velocity estimator data now...")
		arbID = 5
		while True:
			msg = bus.recv()
			if(msg.arbitration_id == arbID and msg.is_extended_id==False):
				v_z_msg = unpack('<f', msg.data[0:4])
				if firstPacket:
					t0 = time.time()
					firstPacket = False
				fp.write("%0.3f"%(time.time()-t0)+", %0.2f\n"%v_z_msg[0])
				if verboseMode:
					print("Vertical Vel: %0.2f"%v_z_msg[0])

	elif sensor == "bno055_att":
		print("Acquiring BNO055 attitude data now...")
		arbID = 11
		while True:
			msg = bus.recv()
			if(msg.arbitration_id == arbID and msg.is_extended_id==False):
				att_msg = unpack('<hhh', msg.data[0:6])
				if firstPacket:
					t0 = time.time()
					firstPacket = False
				fp.write("%0.3f"%(time.time()-t0)+", %0.2f"%att_msg[0]/16.+", %0.2f"%att_msg[1]/16.+", %0.2f\n"%att_msg[2]/16.)
				if verboseMode:
					print("Roll: %0.2f"%att_msg[0]+", Pitch: %0.2f"%att_msg[1]+", Yaw: %0.2f"%att_msg[2])
				
	else:
		print("Invalid sensor ID selected")
	
	fp.close()

if __name__ == '__main__':
	main()