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
import cv2
import depthai as dai
from time import ctime
import csv 




class OakdRGB(Node):
	def __init__(self):
		super().__init__('oakd_rgb')
		self.pipeline = dai.Pipeline()
		xoutVideo = self.pipeline.create(dai.node.XLinkOut)
		xoutVideo.setStreamName("video")
		camRgb = self.pipeline.create(dai.node.ColorCamera)
		camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
		camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
		camRgb.setVideoSize(800, 800)
		camRgb.setFps(20)
		xoutVideo.input.setBlocking(False)
		xoutVideo.input.setQueueSize(1)
		camRgb.video.link(xoutVideo.input)
		self.count = 0
		self.connectDevice()
	
	def connectDevice(self):
	
		with dai.Device(self.pipeline) as device:
			video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
			print('Usb speed: ', device.getUsbSpeed().name)
			while True:
				videoIn = video.get()
				self.count = self.count + 1
				print(str(ctime()) + " - " + str(self.count))
				cv2.imshow("video", videoIn.getCvFrame())
				if cv2.waitKey(1) == ord('q'):
					break

		# Closes all the frames
		cv2.destroyAllWindows()
	
        


def main(args=None):
	rclpy.init(args=args)
	oakdRGB = OakdRGB()
	rclpy.spin(oakdRGB)
	oakdRGB.destroy_node()
	rclpy.shutdown()


	


if __name__ == '__main__':
    main()
