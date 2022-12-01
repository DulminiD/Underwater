from glob import glob
import threading
from zmq import device
import depthai as dai
from time import ctime
import csv
import cv2


f1 = open("log.csv","w")
writer2 = csv.writer(f1)
result = cv2.VideoWriter('filename'+ ctime() + '.avi', 
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
    
def run_RGB():
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
            
            t1.start()
            
            t1.join()
            

if __name__ == "__main__":

    run_RGB()



