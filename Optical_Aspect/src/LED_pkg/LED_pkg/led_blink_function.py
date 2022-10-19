# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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


def main(argv):
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
    GPIO.setmode(GPIO.BOARD)  
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
    main(sys.argv[1:])
