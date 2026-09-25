#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from brushkart_msgs.msg import SensorData

from brushkart_sensor.bno055_driver import BNO055Driver

from brushkart_sensor.ms5803_driver import MS5803Driver

class SensorNode(Node):

    def __init__(self):

        super().__init__('sensor_node')

        self.publisher_ = self.create_publisher(

            SensorData,

            '/sensor_data',

            10

        )

        self.imu = BNO055Driver()

        self.pressure = MS5803Driver()

        self.timer = self.create_timer(

            0.1,

            self.publish_data

        )

    def publish_data(self):

        imu_data = self.imu.get_data()

        pressure_data = self.pressure.get_data()

        msg = SensorData()

        msg.roll_deg = imu_data["roll_deg"]

        msg.pitch_deg = imu_data["pitch_deg"]

        msg.yaw_deg = imu_data["yaw_deg"]

        msg.accel_x = imu_data["accel_x"]

        msg.accel_y = imu_data["accel_y"]

        msg.accel_z = imu_data["accel_z"]

        msg.gyro_x = imu_data["gyro_x"]

        msg.gyro_y = imu_data["gyro_y"]

        msg.gyro_z = imu_data["gyro_z"]

        msg.imu_ok = imu_data["imu_ok"]

        msg.pressure_mbar = pressure_data["pressure_mbar"]

        msg.temperature_c = pressure_data["temperature_c"]

        msg.depth_m = pressure_data["depth_m"]

        msg.pressure_ok = pressure_data["pressure_ok"]

        self.publisher_.publish(msg)

def main():

    rclpy.init()

    node = SensorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()

 