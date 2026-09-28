#!/usr/bin/env python3

import cv2

import time

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from rclpy.qos import (

    QoSProfile,

    ReliabilityPolicy,

    HistoryPolicy

)

class CameraNode(Node):

    def __init__(self):

        super().__init__('camera_node')

        qos = QoSProfile(

            reliability=ReliabilityPolicy.RELIABLE,

            history=HistoryPolicy.KEEP_LAST,

            depth=10

        )

        self.publisher_ = self.create_publisher(

            Image,

            '/camera/raw_image',

            qos

        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():

            self.get_logger().error(

                'Gagal membuka kamera'

            )

            return

        self.frame_counter = 0

        self.last_time = time.time()

        self.saved_debug_frame = False

        self.timer = self.create_timer(

            1.0 / 10.0,

            self.publish_image

        )

        self.get_logger().info(

            'Camera Node Started'

        )

    def publish_image(self):

        ret, frame = self.cap.read()

        if not ret:

            self.get_logger().warning(

                'Gagal membaca frame'

            )

            return

        if frame is None:

            self.get_logger().warning(

                'Frame None'

            )

            return

        mean_pixel = frame.mean()

        self.frame_counter += 1

        now = time.time()

        if now - self.last_time >= 1.0:

            self.get_logger().info(

                f'FPS={self.frame_counter} Mean={mean_pixel:.1f}'

            )

            self.frame_counter = 0

            self.last_time = now

        if not self.saved_debug_frame:

            cv2.imwrite(

                "/tmp/test_ros.jpg",

                frame

            )

            self.saved_debug_frame = True

            self.get_logger().info(

                'Debug image saved: /tmp/test_ros.jpg'

            )

        msg = self.bridge.cv2_to_imgmsg(

            frame,

            encoding='bgr8'

        )

        msg.header.stamp = (

            self.get_clock().now().to_msg()

        )

        msg.header.frame_id = "camera"

        self.publisher_.publish(msg)

    def destroy_node(self):

        self.cap.release()

        super().destroy_node()

def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':

    main()