import cv2
import numpy as np
import imutils
import depthai as dai
from time import ctime
import csv 
import time


lower_blue = np.array([112,125,236,255])
upper_blue = np.array([255,255,253,255])

pipeline = dai.Pipeline()
xoutVideo = pipeline.create(dai.node.XLinkOut)
xoutVideo.setStreamName("video")
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setVideoSize(800, 800)
camRgb.setFps(5)
xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)
camRgb.video.link(xoutVideo.input)
count = 0



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
                print("Light ON")
                cv2.drawContours(image=img, contours=cnts,contourIdx=-1,color=(0,255,0), thickness=2,lineType=cv2.LINE_AA)
            else:
                print("Light Off")
            cv2.imshow('im', img)
            
            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()

            
        

        

	
		

		

		


