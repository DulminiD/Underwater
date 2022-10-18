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

import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.append('~/Desktop/ROS-Tutorials/Node-Test/src/oakd/oakd')

from oakd import oakd_imu_rotation_function 
from oakd import oakd_rgb_function


class OakdRGBIMU(Node):

    def __init__(self):
        super().__init__('oakd_rgb_imu')
   
        

def main(args=None):
	RGB = oakd_rgb_function.OakdRGB();
	IMU = oakd_imu_rotation_function.OakdIMU();
	RGB.main();
	IMU.main();


    


if __name__ == '__main__':
    main()
