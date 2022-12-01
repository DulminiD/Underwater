import Jetson.GPIO as GPIO
import time
import sys
import getopt

# Pin Definitions
output_pin = 12  # BOARD pin 12
frequency = 1


def usage():
    print('gpio_blink.py -f <frequency> -p <output_pin>')
    print('-f or --frequency\t: Frequency to toggle between high and low')
    print('-p or --pin\t: The board pin to output the transmission to')


def blink(argv):
    global output_pin, frequency

    if len(argv) == 1:
        print('Using default values of: Output Pin = Board 12, Frequency = 1 Hz')

    try:
        opts, args = getopt.getopt(argv, "hp:f:", ["help", "pin=", "frequency="])
    except getopt.GetoptError:
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
            sys.exit()
        elif opt in ('-p', '--pin'):
            output_pin = int(arg)
        elif opt in ('-f', '--frequency'):
            frequency = int(arg)

    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # BCM pin-numbering scheme from Raspberry Pi
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.HIGH
    try:
        while True:
            time.sleep((1/frequency))
            # Toggle the output every second
            print("Outputting {} to pin {}".format(curr_value, output_pin))
            GPIO.output(output_pin, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    blink(sys.argv[1:])
