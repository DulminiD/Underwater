import cv2
import os 
import depthai as dai
import keyboard
from time import ctime
from pynput import keyboard

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    
    
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
camRgb.setFps(40)

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

    # Linking
camRgb.video.link(xoutVideo.input)
count = 0

frames = []
video_name = 'generatedvideo.avi'
videoWriter = cv2.VideoWriter(video_name, 0, 1, (800, 800))
image_folder = '/Desktop/OAKD/'

with dai.Device(pipeline) as device:

        video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
        print('Usb speed: ', device.getUsbSpeed().name)

        while True:
            videoIn = video.get()
            frames.append(videoIn) 
            count = count + 1 

            print(str(ctime()) + " - " + str(count))

            name = str(count) + ".jpg"


            # cv2.imshow("name", vidbeoIn.gebtCvFrame())
            # key = cv2.waitKey(1)

            # if key == ord('q'):
            #     # Quit when q is pressed
            #     break
            # elif key == ord('t'):
            #     print('TTTTTt')

            if cv2.waitKey(0) & 0xFF == ord('q'):
                break
                # print('Inside')
                # for image in frames: 
                #     video.write(cv2.imread(os.path.join(image_folder, image))) 
                # print('Done')
        
