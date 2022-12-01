import csv
import binascii
from datetime import *
from crc import CrcCalculator, Crc8

import Jetson.GPIO as GPIO

import signal
import time
import logging
import getopt
import random
import sys

output_pin = 11  # Board Pin 12
frequency = 25  # effectively a 30Hz transmission rate
# variables for random bit transmission
random_flag = True
random_size = 1040
times = 1
# variables for perma state transmission
state_flag = False
perma_state = GPIO.LOW

# storing a list of stuff to log so that we can put it after transmission has ended
# This is to maximise performance to get close to the 25KHz as the theoretical upper limit
output_name = 'transmitter_{0}Hz_{1}'.format(frequency, datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
logs = list()

def interrupt_handler(sig, frame):
	print("You've pressed Ctrl+C!")
	log_data()
	logging.info("Program ending", extra={'time': time.perf_counter()})
	GPIO.cleanup(output_pin)
	sys.stdout.flush()
	sys.exit(0)


def progress_bar(percent_done, bar_length=50):
	done_length = int(bar_length * percent_done / 100)
	bar = '=' * done_length + '-' * (bar_length - done_length)
	sys.stdout.write('[%s] %f%s\r' % (bar, percent_done, '%'))
	#if(int(percent_done)/100 == 1):
		
	sys.stdout.flush()


def transmission_mask(bits):
	transmission = []

    # start transmission mask
	for i in range(3):
		transmission.append(1)

	for i in range(2):
		transmission.append(1)

    # transmission sequence
	for bit in bits:
		transmission.append(int(bit))

    # transmission parity 
	transmission.append(0)

    # transmission CRC
	crc = create_crc(bits)
	for bit in crc:
		transmission.append(bit)
    

    # transmission end mask
	transmission.append(1)
	transmission.append(0)
	transmission.append(0)


	return transmission


def create_crc(bits):
	crc = []
	val = 0
	v1 = 0
	expected_checksum = 0xBC
	crc_calculator = CrcCalculator(Crc8.CCITT)
	checksum = crc_calculator.calculate_checksum(bits)
	for d in str(checksum):
		val = val + int(d)
	for d in str(val):
		v1 = v1 + int(d)

	if v1 > 7:
		v1 = 7

	v = "{0:b}".format(v1)
	checksum = [int(d) for d in str(v)]
	if len(checksum) < 3:
		crc.append(0)
	for c in checksum:
		crc.append(c)
	return crc


def create_sequence(sequence):
	bit_array = transmission_mask(sequence)

	return bit_array


def log_data():
	for deltaTime, value in logs:
		logging.info("Transmitted: {0}".format(value), extra={'time': deltaTime})

	with open(output_name + "_data.csv", mode='w') as csv_file:
		csv_file = csv.writer(csv_file)
		csv_file.writerow(['deltaTime', 'bit'])
	for deltaTime, value in logs:
		csv_file.writerow([deltaTime, value])

def log_transmission(diff_time, bit):
	with open(output_name + "_data1.csv", mode='a') as csv_file:
		csv_file = csv.writer(csv_file)
		csv_file.writerow([diff_time,bit])


def transmit(transmission_bits, start_time):
	try:
        # Pin Setup:
		GPIO.setmode(GPIO.BOARD)

        # set pin as an output pin with initial state low
		GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
		for bit in transmission_bits:
			GPIO.output(output_pin, int(bit))
			log_transmission(datetime.now().strftime('%Y-%m-%d %H:%M:%S'), bit)
			current_time = time.perf_counter()
			diff_time = current_time - start_time
			logs.append((diff_time, bit))
			# progress_bar(float(len(logs) / len(transmission_bits)) * 100, 30)
			time.sleep(1/frequency)
		#print("Packet Done!")
	finally:
		print("Packet Done!")
		# GPIO.output(output_pin, GPIO.LOW)
		# GPIO.cleanup(output_pin)

		# log_data()


def usage():
	print('transmitter_randombits_repeat.py -s <define_static_state> -r <length_of_random_bitstream> -f '
          '<frequency_to_transmit> -t <number_of_times>')
	print('-f or --frequency\t: Frequency to transmit at')
	print('-r or --random\t: Flag to set random bit transmission followed by the size of the bitstream')
	print('-t or --times\t: Number of times the random bitstream is to be repeated')
	print('-s or --state\t: Set the permanent state of the LED. This flag takes precedence so include only for LED '
          'ON/OFF experiment')


def get_perma_state(input_state):
	if input_state in ('ON', 'on'):
		state = GPIO.HIGH
	elif input_state in ('OFF', 'off'):
		state = GPIO.LOW
	else:
		usage()
		print('Expected values for state flag are: ON/OFF')
		sys.exit(2)

	return state


def generate_random_bitstream(size):
	bitstream = []
	for i in range(size):
		bitstream.append(random.randint(0, 1))

	return bitstream


def main(argv):
	run_stream(argv)
	
def run_stream(argv):
	global output_pin, frequency, state_flag, perma_state, random_flag, random_size, times, output_name
	if len(argv) == 1:
		print('Using default values of: Output Pin = Board 12, Frequency = 30 Hz, 500 random bits and 1 cycle')
	try:
		opts, args = getopt.getopt(argv, "hs:r:f:t:", ["state=", "random=", "frequency=", "times="])
	except getopt.GetoptError:
		usage()
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-h':
			usage()
			sys.exit()
		elif opt in ('-s', '--state'):
			state_flag = True
			perma_state = get_perma_state(arg)
		elif opt in ('-f', '--freq'):
			frequency = int(arg)
		elif opt in ('-r', '--random'):
			random_flag = True
			random_size = int(arg)
		elif opt in ('-t', '--times'):
			times = int(arg)

    # ToDo: Change the filename here if in the future our datasets change
	output_name = 'transmitter_{0}Hz_{1}_cycles-{2}_bits'.format(frequency, times, random_size)
	print(output_name)

	signal.signal(signal.SIGINT, interrupt_handler)

	if state_flag:
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(output_pin, GPIO.OUT, initial=perma_state)
		GPIO.output(output_pin, perma_state)
		print('Output set to permanent state: {0}'.format(perma_state))
		GPIO.cleanup(output_pin)
	elif random_flag:
        # logging config
		logging.basicConfig(filename=output_name + ".log",
                            level=logging.INFO, format='%(time)s: %(message)s')
		random_bits = generate_random_bitstream(random_size)
		currentTime = time.perf_counter()
		# logging.info("Generated bitstream: {0}".format(random_bits), extra={'time': currentTime})

		for i in range(0,random_size):
			bitstream_for_sequence = random_bits[i:i+52]
			transmission = transmission_mask(bitstream_for_sequence)

			print("Transmitting")
			start_time = time.perf_counter()
			# logging.info("Starting Transmission", extra={'time': start_time})
			transmit(transmission, start_time)
			# logging.info("Transmission Ended", extra={'time': time.perf_counter()})
			i += 52

        
	else:
		print('No flags set, exiting')
		usage()
		sys.exit(0)


if __name__ == '__main__':
	main(sys.argv[1:])
