#!/usr/bin/env python3

import cv2
import depthai as dai
import rclpy
from rclpy.node import Node


class OakdStereo(Node):
	def __init__(self):
	
		# Create pipeline
		self.pipeline = dai.Pipeline()

		# Define sources and outputs
		monoLeft = self.pipeline.create(dai.node.MonoCamera)
		monoRight = self.pipeline.create(dai.node.MonoCamera)
		xoutLeft = self.pipeline.create(dai.node.XLinkOut)
		xoutRight = self.pipeline.create(dai.node.XLinkOut)

		xoutLeft.setStreamName('left')
		xoutRight.setStreamName('right')

		# Properties
		monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
		monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
		monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
		monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

		# Linking
		monoRight.out.link(xoutRight.input)
		monoLeft.out.link(xoutLeft.input)
		self.connectDevice()
		
	def connectDevice(self):
		
		with dai.Device(self.pipeline) as device:

		    qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
		    qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)

		    while True:
		    	inLeft = qLeft.tryGet()
		    	inRight = qRight.tryGet()
		    	if inLeft is not None:
		    		cv2.imshow("left", inLeft.getCvFrame())
		    	if inRight is not None:
		    		cv2.imshow("right", inRight.getCvFrame())
		    	if cv2.waitKey(1) == ord('q'):
		    		break
			
def main(args=None):
	rclpy.init(args=args)
	oakdStereo = OakdStereo()
	rclpy.spin(oakdStereo)
	oakdStereo.destroy_node()
	rclpy.shutdown()


    


if __name__ == '__main__':
    main()
