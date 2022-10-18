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




class OakdPublisher(Node):

	def __init__(self):
		super().__init__('oakd_publisher')
		self.pipeline = dai.Pipeline()

		camRgb = self.pipeline.create(dai.node.ColorCamera)
		imu = self.pipeline.create(dai.node.IMU)

		xoutVideo = self.pipeline.create(dai.node.XLinkOut)
		xlinkOut = self.pipeline.create(dai.node.XLinkOut)

		xoutVideo.setStreamName("video")
		xlinkOut.setStreamName("imu")

		# Properties
		camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
		camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
		# THE_4_K
		camRgb.setVideoSize(800, 800)
		camRgb.setFps(30)


		#IMU settings 
		imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 30)
		imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 30)
		imu.setBatchReportThreshold(1)
		imu.setMaxBatchReports(10)

		xoutVideo.input.setBlocking(False)
		xoutVideo.input.setQueueSize(1)

		# Linking
		camRgb.video.link(xoutVideo.input)
		imu.out.link(xlinkOut.input)

		self.count = 0
		frameArray = []

		f = open("log.csv","w")
		self.writer = csv.writer(f)


		self.result = cv2.VideoWriter('filename.avi', 
				         cv2.VideoWriter_fourcc(*'MJPG'),
				         30, (800,800))
		self.connectDevice()
				         
		                
	def connectDevice(self):
	
		with dai.Device(self.pipeline) as device:

		    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
		    print('Usb speed: ', device.getUsbSpeed().name)
		    
		    def timeDeltaToMilliS(delta) -> float:
		    	return delta.total_seconds()*1000


		    # Output queue for imu bulk packets
		    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
		    baseTs = None

		    while True:
		    	videoIn = video.get()
		    	imuData = imuQueue.get()
		    	self.count = self.count + 1
		    	print(str(ctime()) + " - " + str(self.count))
		    	frame = videoIn.getCvFrame()
		    	self.result.write(frame)
		    	cv2.imshow("video", frame)
		    	imuPackets = imuData.packets
		    	for imuPacket in imuPackets:
		    		acceleroValues = imuPacket.acceleroMeter
		    		gyroValues = imuPacket.gyroscope
		    		acceleroTs = acceleroValues.timestamp.get()
		    		gyroTs = gyroValues.timestamp.get()
		    		if baseTs is None:
		    			baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
		    		acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
		    		gyroTs = timeDeltaToMilliS(gyroTs - baseTs)
		    		imuF = "{:.06f}"
		    		tsF  = "{:.03f}"
		    		
		    		row = [str(ctime()) + " - " + str(self.count), {tsF.format(acceleroTs)} , {imuF.format(acceleroValues.x)}, {imuF.format(acceleroValues.y)}, {imuF.format(acceleroValues.z)}, {tsF.format(gyroTs)}, {imuF.format(gyroValues.x)}, {imuF.format(gyroValues.y)}, {imuF.format(gyroValues.z)} ]
		    		self.writer.writerow(row)
		    		print(str(ctime()) + " - " + str(self.count), {tsF.format(acceleroTs)} , {imuF.format(acceleroValues.x)}, {imuF.format(acceleroValues.y)}, {imuF.format(acceleroValues.z)}, {tsF.format(gyroTs)}, {imuF.format(gyroValues.x)}, {imuF.format(gyroValues.y)}, {imuF.format(gyroValues.z)}) 
		    		print(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
		    		print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
		    		print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
		    		print(f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")
		    		
		    	if cv2.waitKey(1) == ord('q'):
		    		break

		result.release()
		    
		# Closes all the frames
		cv2.destroyAllWindows()

				        

        
 
def main(args=None):

	rclpy.init(args=args)
	oakd_all = OakdPublisher();
	rclpy.spin(oakd_all)
	rclpy.shutdown()
	

if __name__ == '__main__':
    main()
