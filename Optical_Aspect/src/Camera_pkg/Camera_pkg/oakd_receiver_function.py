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


import cv2
import numpy as np
import imutils
import depthai as dai
from time import ctime
import csv 
import time

f = open("Received.csv","w")
writer = csv.writer(f)
sensitivity = 15
lower_blue = np.array([0,0,255-sensitivity])
upper_blue = np.array([255,sensitivity,255])


pipeline = dai.Pipeline()
xoutVideo = pipeline.create(dai.node.XLinkOut)
xoutVideo.setStreamName("video")
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setVideoSize(800, 800)
camRgb.setFps(50)
xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)
camRgb.video.link(xoutVideo.input)
count = 0
status = 'OFF'



if __name__ == "__main__":

    with dai.Device(pipeline) as device:
        video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
        print('Usb speed: ', device.getUsbSpeed().name)
        while True:
            videoIn = video.get()
            count = count + 1
            print(str(ctime()) + " - " + str(count))
            img = videoIn.getCvFrame()
            
            blurred = cv2.GaussianBlur(img, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                status = 'ON'
                cv2.drawContours(image=img, contours=cnts,contourIdx=-1,color=(0,255,0), thickness=2,lineType=cv2.LINE_AA)
            else:
                status = 'OFf'
            cv2.imshow('im', img)
            writer.writerow(status)
            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()

            
        

  


