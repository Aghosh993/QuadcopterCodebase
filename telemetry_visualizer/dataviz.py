import os
import serial
import matplotlib

message_id_state = 1
message_id_sensor_data = 2

class State_DataPoint:
	def __init__(self, timestamp, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, x, y, z, v_x, v_y, v_z, a_x, a_y, a_z):
		self.timestamp = timestamp
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.roll_rate = roll_rate
		self.pitch_rate = pitch_rate
		self.yaw_rate = yaw_rate
		self.x = x
		self.y = y
		self.z = z
		self.v_x = v_x
		self.v_y = v_y
		self.v_z = v_z
		self.a_x = a_x
		self.a_y = a_y
		self.a_z = a_z

class Sensor_DataPoint:
	def __init__(self, timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, optical_flow_x_vel, optical_flow_y_vel, optical_flow_sonar_height, lidar_height, gps_lat, gps_long, gps_altitide):
		self.timestamp = timestamp
		self.gyro_x = gyro_x
		self.gyro_y = gyro_y
		self.gyro_z = gyro_z
		self.accel_x = accel_x
		self.accel_y = accel_y
		self.accel_z = accel_z
		self.mag_x = mag_x
		self.mag_y = mag_y
		self.mag_z = mag_z
		self.optical_flow_x_vel = optical_flow_x_vel
		self.optical_flow_y_vel = optical_flow_y_vel
		self.optical_flow_sonar_height = optical_flow_sonar_height
		self.lidar_height = lidar_height
		self.gps_lat = gps_lat
		self.gps_long = gps_long
		self.gps_altitide = gps_altitide

class CommandsActuations_DataPoint:
	def __init__(self, roll_cmd, pitch_cmd, yaw_cmd, height_cmd, m1_throttle, m2_throttle, m3_throttle, m4_throttle):
		self.roll_cmd = roll_cmd
		self.pitch_cmd = pitch_cmd
		self.yaw_cmd = yaw_cmd
		self.height_cmd = height_cmd
		self.m1_throttle = m1_throttle
		self.m2_throttle = m2_throttle
		self.m3_throttle = m3_throttle
		self.m4_throttle = m4_throttle

def main():
	global message_id_state
	global message_id_sensor_data

	try:
		ser = serial.Serial("/dev/ttyUSB3", 115200)
	except:
		print("Unable to open port. Maybe a permissions/availability issue?")
		exit()
	
	state_data_set = []
	sensor_data_set = []

	ser.flush()

	while(True):
		start_byte = ser.read(1)

		if start_byte == 's':

			message_id = ser.read(1)

			if message_id == message_id_state:

				data = ser.read(4*16)

			if message_id == message_id_sensor_data:

				data = ser.read(4*17)

if __name__ == '__main__':
	main()