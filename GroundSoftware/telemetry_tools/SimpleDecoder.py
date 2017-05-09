import os
import sys
import serial
from time import sleep
from struct import *

class SimpleProtocolDecoder:
	def __init__(self, port, baud):
		self.serport = serial.Serial(port, baud)

	def get_string_message(self):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			descriptor = self.serport.read(desc_size)
			sys.stdout.write(descriptor)
			sys.stdout.write(": ")

			msg_id = self.serport.read(1)
			data_len = ord(self.serport.read(1))
			msg_payload = self.serport.read(data_len)
			chksum = self.serport.read(2)

			sys.stdout.write("DATA:")
			sys.stdout.write(msg_payload)
			sys.stdout.write("\n")

	def get_n_ints_message(self):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			descriptor = self.serport.read(desc_size)
			sys.stdout.write(descriptor)
			sys.stdout.write(": ")

			msg_id = self.serport.read(1)
			print("MSG ID: "+repr(ord(msg_id)))
			data_len = ord(self.serport.read(1))
			print("LEN: "+repr(data_len))
			
			int_list = []
			for i in range(data_len):
				msg_payload_segment = self.serport.read(4)	
				int_list.append(unpack('<i', msg_payload_segment)[0])
			
			print("DATA:")
			print(int_list)

			chksum = self.serport.read(2)
			sys.stdout.write("\n")

	def get_n_floats_message(self):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			descriptor = self.serport.read(desc_size)
			sys.stdout.write(descriptor)
			sys.stdout.write(": ")

			msg_id = self.serport.read(1)
			print("MSG ID: "+repr(ord(msg_id)))
			data_len = ord(self.serport.read(1))
			print("LEN: "+repr(data_len))
			
			float_list = []
			for i in range(data_len):
				msg_payload_segment = self.serport.read(4)	
				float_list.append(unpack('<f', msg_payload_segment)[0])
			
			print("DATA:")
			print(float_list)
			
			chksum = self.serport.read(2)
			sys.stdout.write("\n")

	def get_m_n_float_matrix_message(self):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			descriptor = self.serport.read(desc_size)
			sys.stdout.write(descriptor)
			sys.stdout.write(": ")

			msg_id = self.serport.read(1)
			print("MSG ID: "+repr(ord(msg_id)))
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

	def get_m_n_int_matrix_message(self):
		st = self.serport.read(1)
		if st == 's':
			desc_size = ord(self.serport.read(1))
			descriptor = self.serport.read(desc_size)
			sys.stdout.write(descriptor)
			sys.stdout.write(": ")

			msg_id = self.serport.read(1)
			print("MSG ID: "+repr(ord(msg_id)))
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

def main():

	s = SimpleProtocolDecoder(sys.argv[1], 460800)
	# s.get_string_message()
	# s.get_n_ints_message()
	# s.get_n_floats_message()
	# s.get_m_n_float_matrix_message()
	# s.get_m_n_int_matrix_message()
	while True:
		s.get_n_ints_message()

if __name__ == '__main__':
	main()