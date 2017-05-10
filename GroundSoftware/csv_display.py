#!/usr/bin/python3
import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt

def main():
	parser = argparse.ArgumentParser(description='Display a CSV file')
	parser.add_argument('filename', metavar='filename', nargs=1, help='CSV file to open')

	args = parser.parse_args()

	fn = args.filename[0]

	input_data = np.loadtxt(fn, delimiter=', ')
	

if __name__ == '__main__':
	main()