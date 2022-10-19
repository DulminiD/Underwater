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

output_pin = 12  # Board Pin 12
frequency = 30  # effectively a 30Hz transmission rate
sequence = [1,1,1,0,0,1,0,0]

def interrupt_handler(sig, frame):
    print("You've pressed Ctrl+C!")
    logging.info("Program ending")
    GPIO.cleanup(output_pin)
    sys.exit(0)


def transmission_mask(bits):
    transmission = []

    # start transmission mask
    for i in range(3):
        transmission.append(1)

    for i in range(2):
        transmission.append(1)

    # transmission sequence
    bits = bits * 6
    for bit in bits:
        transmission.append(int(bit))
    for i in range(4):
        transmission.append(0)

    # transmission parity 
    transmission.append(0)

    # transmission CRC
    crc = create_crc(bits)
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

def create_sequence(sequence):
    bit_array = transmission_mask(sequence)

    return bit_array


def create_transmission(sequence):
    transmission = create_sequence(sequence)  
    return transmission  # multiplies it by the number of times to be repeated


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


def usage():
    print('transmission.py -p <output_pin> -f <frequency_to_transmit> -m <sequence_to_send>')
    print('-p or --pin\t: Pin to transmit out to')
    print('-f or --frequency\t: Frequency to transmit at')
    print('-m or --sequence\t: sequence to send')


def main(argv):
    global output_pin, frequency, sequence
    if len(argv) == 1:
        print('Using default values of: Output Pin = Board 12, Frequency = 30 Hz')
    try:
        opts, args = getopt.getopt(argv, "hp:f:m:", ["pin=", "freq=", "sequence="])
    except getopt.GetoptError:
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            usage()
            sys.exit()
        elif opt in ('-p', '--pin'):
            output_pin = arg
        elif opt in ('-f', '--freq'):
            frequency = int(arg)
        elif opt in ('-m', '--sequence'):
            sequence = arg
            sequence = list(map(int, sequence.split(" ")))
            

    # logging config
    logging.basicConfig(filename='transmitter-{0}.log'.format(datetime.now().strftime('%m-%d-%Y-%H:%M:%S')), level=logging.INFO, format='%(asctime)s %(sequence)s')
    signal.signal(signal.SIGINT, interrupt_handler)

    transmission = create_transmission(sequence)
    print("Transmitting")
    logging.info("Starting Transmission")
    transmit(transmission)


if __name__ == '__main__':
    main(sys.argv[1:])