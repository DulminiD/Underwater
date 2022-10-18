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
import threading
from zmq import device

baseTs = None
f = open("IMU.csv","w")
f1 = open("log.csv","w")
writer = csv.writer(f)
writer2 = csv.writer(f1)
result = cv2.VideoWriter('filename.avi', 
		                 cv2.VideoWriter_fourcc(*'MJPG'),
		                 55, (800,800))

def timeDeltaToMilliS(delta) -> float:
			return delta.total_seconds()*1000

def methodOne(videoQueue, count):
    global writer2

    # oakd_rgb_function.RGB(videoQueue);
    videoIn = videoQueue.get()
    count = count + 1
    writer2.writerow(str(ctime()) + " - " + str(count))
    print(str(ctime()) + " - " + "Video Video")
    result.write(videoIn.getCvFrame())
    

def methodTwo(imuQueue, count):
    global baseTs
    global writer
   
    # oakd_imu_rotation_function.IMU(imuQueue);
    imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived
    # count = count + 1
    imuPackets = imuData.packets
    for imuPacket in imuPackets:
        print('IMU IMU ' , len(imuPackets))
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

        row = [str(ctime()) + " - " , count, {tsF.format(acceleroTs)} , {imuF.format(acceleroValues.x)}, {imuF.format(acceleroValues.y)}, {imuF.format(acceleroValues.z)}, {tsF.format(gyroTs)}, {imuF.format(gyroValues.x)}, {imuF.format(gyroValues.y)}, {imuF.format(gyroValues.z)} ]
        writer.writerow(row)

        print(str(ctime()) + " - " + "IMU IMU", {tsF.format(acceleroTs)} , {imuF.format(acceleroValues.x)}, {imuF.format(acceleroValues.y)}, {imuF.format(acceleroValues.z)}, {tsF.format(gyroTs)}, {imuF.format(gyroValues.x)}, {imuF.format(gyroValues.y)}, {imuF.format(gyroValues.z)}) 

def main(args=None):

    count = 0

    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    camRgb = pipeline.create(dai.node.ColorCamera)


    xoutVideo = pipeline.create(dai.node.XLinkOut)
    xoutVideo.setStreamName("video")
    xlinkOut = pipeline.create(dai.node.XLinkOut)
    xlinkOut.setStreamName("imu")
    
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 55)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 55)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    imu.out.link(xlinkOut.input)


    camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setVideoSize(800, 800)
    camRgb.setFps(58)
    xoutVideo.input.setBlocking(False)
    xoutVideo.input.setQueueSize(1)
    camRgb.video.link(xoutVideo.input)

    with dai.Device(pipeline) as device:

        imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        videoQueue = device.getOutputQueue(name="video", maxSize=1, blocking=False)

        while True:

            count = count + 1
            t1 = threading.Thread(target=methodOne(videoQueue, count))
            t2 = threading.Thread(target=methodTwo(imuQueue, count))
            
            t1.start()
            t2.start()
            
            t1.join()
            t2.join()
            

if __name__ == '__main__':
    main()



