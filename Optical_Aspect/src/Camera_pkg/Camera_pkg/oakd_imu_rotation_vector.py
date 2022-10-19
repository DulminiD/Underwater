#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
import time
import math


class OakdIMURotation(Node):

    def __init__(self):
        super().__init__('oakd_imu_rotation')
        self.pipeline = dai.Pipeline()
        self.imu = self.pipeline.create(dai.node.IMU)
        self.xlinkOut = self.pipeline.create(dai.node.XLinkOut)
        self.xlinkOut.setStreamName("imu")
        self.imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
        self.imu.setBatchReportThreshold(1)
        self.imu.setMaxBatchReports(10)
        self.imu.out.link(self.xlinkOut.input)
        self.connectDevice()

    def connectDevice(self):

        with dai.Device(self.pipeline) as device:

            def timeDeltaToMilliS(delta) -> float:
                return delta.total_seconds()*1000

            # Output queue for imu bulk packets
            imuQueue = device.getOutputQueue(
                name="imu", maxSize=50, blocking=False)
            baseTs = None
            while True:
                imuData = imuQueue.get()
                imuPackets = imuData.packets
                for imuPacket in imuPackets:
                    rVvalues = imuPacket.rotationVector
                    rvTs = rVvalues.timestamp.get()
                    if baseTs is None:
                        baseTs = rvTs
                    rvTs = rvTs - baseTs
                    imuF = "{:.06f}"
                    tsF = "{:.03f}"

                    print(
                        f"Rotation vector timestamp: {tsF.format(timeDeltaToMilliS(rvTs))} ms")
                    print(
                        f"Quaternion: i: {imuF.format(rVvalues.i)} j: {imuF.format(rVvalues.j)} ")
                    print(
                        f"k: {imuF.format(rVvalues.k)} real: {imuF.format(rVvalues.real)}")
                    print(
                        f"Accuracy (rad): {imuF.format(rVvalues.rotationVectorAccuracy)}")

                if cv2.waitKey(1) == ord('q'):
                    break


def main(args=None):

    rclpy.init(args=args)
    oakd_imu_rotation = OakdIMURotation()
    rclpy.spin(oakd_imu_rotation)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
