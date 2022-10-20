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


class OakdIMU(Node):

    def __init__(self):
        super().__init__('oakd_imu')
        self.pipeline = dai.Pipeline()
        self.imu = self.pipeline.create(dai.node.IMU)
        self.xlinkOut = self.pipeline.create(dai.node.XLinkOut)
        self.xlinkOut.setStreamName("imu")
        self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 60)
        self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 60)
        self.imu.setBatchReportThreshold(1)
        self.imu.setMaxBatchReports(10)
        self.imu.out.link(self.xlinkOut.input)
        self.count = 0
        self.f = open("./output/IMU_rotation_log_"+str(ctime())+".csv", "w")
        self.writer = csv.writer(self.f)
        self.connectDevice()

    def connectDevice(self):
        # Pipeline is defined, now we can connect to the device
        with dai.Device(self.pipeline) as device:

            def timeDeltaToMilliS(delta) -> float:
                return delta.total_seconds()*1000

            # Output queue for imu bulk packets
            imuQueue = device.getOutputQueue(
                name="imu", maxSize=50, blocking=False)
            baseTs = None
            while True:
                imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived
                self.count = self.count + 1
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
                    tsF = "{:.03f}"

                    row = [str(ctime()) + " - " + str(self.count), {tsF.format(acceleroTs)}, {imuF.format(acceleroValues.x)}, {imuF.format(acceleroValues.y)}, {
                        imuF.format(acceleroValues.z)}, {tsF.format(gyroTs)}, {imuF.format(gyroValues.x)}, {imuF.format(gyroValues.y)}, {imuF.format(gyroValues.z)}]
                    self.writer.writerow(row)

                    print(str(ctime()) + " - " + str(self.count), {tsF.format(acceleroTs)}, {imuF.format(acceleroValues.x)}, {imuF.format(acceleroValues.y)}, {
                          imuF.format(acceleroValues.z)}, {tsF.format(gyroTs)}, {imuF.format(gyroValues.x)}, {imuF.format(gyroValues.y)}, {imuF.format(gyroValues.z)})
                    print(
                        f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
                    print(
                        f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
                    print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
                    print(
                        f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")

                if cv2.waitKey(1) == ord('q'):
                    break

        # Closes all the frames
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    oakd_imu = OakdIMU()
    rclpy.spin(oakd_imu)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
