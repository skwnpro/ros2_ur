#!/usr/bin/env python3

import cv2

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class CameraNode(Node):

    def __init__(self):

        super().__init__('camera_node')

        self.publisher_ = self.create_publisher(

            Image,

            '/camera/raw_image',

            10

        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():

            self.get_logger().error(

                'Gagal membuka /dev/video0'

            )

            return

        self.cap.set(

            cv2.CAP_PROP_FRAME_WIDTH,

            1280

        )

        self.cap.set(

            cv2.CAP_PROP_FRAME_HEIGHT,

            720

        )

        self.timer = self.create_timer(

            1.0 / 20.0,

            self.publish_image

        )

        self.get_logger().info(

            'Camera Node Started'

        )

    def publish_image(self):

        ret, frame = self.cap.read()

        if not ret:

            self.get_logger().warning(

                'Frame gagal dibaca'

            )

            return

        msg = self.bridge.cv2_to_imgmsg(

            frame,

            encoding='bgr8'

        )

        msg.header.stamp = \
            self.get_clock().now().to_msg()

        msg.header.frame_id = "camera"

        self.publisher_.publish(msg)

    def destroy_node(self):

        self.cap.release()

        super().destroy_node()

def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()