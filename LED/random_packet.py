import binascii
from datetime import *
from msilib import sequence
from crc import CrcCalculator, Crc8

import Jetson.GPIO as GPIO

import signal
import sys
import time
import logging
import getopt
import random


output_pin = 12  # Board Pin 12
frequency = 30 
random_size = 200

def interrupt_handler(sig, frame):
    print("You've pressed Ctrl+C!")
    logging.info("Program ending")
    GPIO.cleanup(output_pin)
    sys.exit(0)

def generate_random_bitstream(size):
    bitstream = []
    for i in range(size):
        bitstream.append(random.randint(0, 1))

    return bitstream


def create_transmission(sequence):
    global random_size

    transmission = []

    # start transmission mask
    for i in range(3):
        transmission.append(1)

    for i in range(2):
        transmission.append(1)

    # sequence 
    for bit in sequence:
        transmission.append(int(bit))
    for i in range(4):
        transmission.append(0)

    # transmission parity 
    transmission.append(0)

    # transmission CRC
    crc = create_crc(sequence)
    transmission.append(crc)
    

    # transmission end mask
    transmission.append(1)
    transmission.append(0)
    transmission.append(0)

    return transmission

def create_crc(bits):
    crc = []
    expected_checksum = 0xBC
    crc_calculator = CrcCalculator(Crc8.CCITT)
    checksum = crc_calculator.calculate_checksum(bits)
    checksum = [int(d) for d in str(checksum)]
    crc.append(0)
    crc.append(checksum)

    return crc


def transmit(transmission_bits):
    try:
        # Pin Setup:
        GPIO.setmode(GPIO.BOARD)  # BCM pin-numbering as in Raspberry Pi

        # set pin as an output pin with initial state low
        GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
        for bit in transmission_bits:
            GPIO.output(output_pin, bit)
            logging.info("Transmitted: {0}".format(bit))
            time.sleep(1/frequency)
    finally:
        GPIO.cleanup(output_pin)




def main():

    # logging config
    logging.basicConfig(filename='transmitter-{0}.log'.format(datetime.now().strftime('%m-%d-%Y-%H:%M:%S')), level=logging.INFO, format='%(asctime)s %(sequence)s')
    signal.signal(signal.SIGINT, interrupt_handler)

    # Generate the random bit stream 
    sequence = generate_random_bitstream(random_size)

    for i in random_size/50:

        # Create the transmission 
        transmission = create_transmission(sequence[i:i+49])
        print("Transmitting")
        logging.info("Starting Transmission")
        transmit(transmission)


if __name__ == '__main__':
    main(sys.argv[1:])
