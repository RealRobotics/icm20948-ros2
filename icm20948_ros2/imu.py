#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2023 University of Leeds
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField
from pipebot_msgs.msg import ImuCalibrationStatus
from std_srvs.srv import Empty

from imu_bno055.bno055 import BNO055_I2C

PUBLISH_RATE = 20  # per second
PUBLISH_INTERVAL_S = 1 / PUBLISH_RATE

SLOW_PUBLISH_INTERVAL_S = 1.0


class ImuNode(Node):
    def __init__(self, imu):
        super().__init__("imu")
        self.get_logger().info("IMU has started!")
        self.__imu = imu

        # Set up Publishers
        self.imu_publisher_ = self.create_publisher(Imu, "imu/imu", 10)
        self.imu_timer_ = self.create_timer(PUBLISH_INTERVAL_S, self.publish_imu)

        self.imu_raw_publisher_ = self.create_publisher(Imu, "imu/imu_raw", 10)
        self.imu_raw_timer_ = self.create_timer(
            PUBLISH_INTERVAL_S, self.publish_imu_raw
        )

        self.imu_temp_publisher_ = self.create_publisher(Temperature, "imu/temp", 10)
        self.imu_temp_timer_ = self.create_timer(
            SLOW_PUBLISH_INTERVAL_S, self.publish_imu_temp
        )

        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", 10)
        self.imu_mag_timer_ = self.create_timer(
            PUBLISH_INTERVAL_S, self.publish_imu_mag
        )

        self.imu_calib_status_publisher_ = self.create_publisher(
            ImuCalibrationStatus, "imu/calib_status", 10
        )
        self.imu_calib_timer_ = self.create_timer(
            PUBLISH_INTERVAL_S, self.publish_imu_calib_status
        )

        # Set up Servers
        self.imu_calibrate_server = self.create_service(
            Empty, "imu/calibrate", self.callback_imu_calibrate
        )

    #    def publish_diagnostics(self):
    #        msg = DiagnosticArray()
    #        # self.get_logger().info("pub diagnostics")
    #        self.diagnostics_publisher_.publish(msg)

    def publish_imu(self):
        msg = self.__imu.imu()
        self.imu_publisher_.publish(msg)

    def publish_imu_raw(self):
        msg = self.__imu.imu_raw()
        self.imu_raw_publisher_.publish(msg)

    def publish_imu_temp(self):
        msg = self.__imu.temperature()
        # self.get_logger().info("pub imu_temp")
        self.imu_temp_publisher_.publish(msg)

    def publish_imu_mag(self):
        msg = self.__imu.magnetic()
        self.imu_mag_publisher_.publish(msg)

    def publish_imu_calib_status(self):
        msg = self.__imu.calibration_status()
        self.imu_calib_status_publisher_.publish(msg)

    def callback_imu_calibrate(self, request, response):
        self.get_logger().info("Calibrate service requested")
        # TODO(AJB): Add real service.
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    imu = BNO055_I2C()
    node = ImuNode(imu)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
