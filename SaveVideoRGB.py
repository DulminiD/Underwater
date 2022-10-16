#!/usr/bin/env python3

import cv2
import depthai as dai
from time import ctime
import os.path

save_path = '/home/mcpslab/Desktop/OAKD/NotMoving'

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
# THE_4_K
camRgb.setVideoSize(800, 800)
camRgb.setFps(50)

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

# Linking
camRgb.video.link(xoutVideo.input)
count = 0
frameArray = []

result = cv2.VideoWriter('filename.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         50, (800,800))

# print('Usb speed: ', device.getUsbSpeed().name)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
    print('Usb speed: ', device.getUsbSpeed().name)

    while True:
        videoIn = video.get()
        count = count + 1
        # w = str(ctime()) + " - " + str(count)
        print(str(ctime()) + " - " + str(count))
        frame = videoIn.getCvFrame()
        result.write(frame)
        # frameArray.append(frame)
        cv2.imshow("video", frame)
        # name = str(count) + ".jpg"
        # completeName = os.path.join(save_path, name)
        # cv2.imwrite(completeName, videoIn.getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break



result.release()
    
# Closes all the frames
cv2.destroyAllWindows()

