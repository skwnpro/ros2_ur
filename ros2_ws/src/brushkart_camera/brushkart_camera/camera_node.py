#!/usr/bin/env python3

import cv2

import time

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import (

    Image,

    CompressedImage

)

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

        #

        # RAW IMAGE

        #

        self.raw_publisher = self.create_publisher(

            Image,

            '/camera/raw_image',

            qos

        )

        #

        # COMPRESSED IMAGE

        #

        self.compressed_publisher = self.create_publisher(

            CompressedImage,

            '/camera/compressed',

            qos

        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(

            0,

            cv2.CAP_V4L2

        )

        if not self.cap.isOpened():

            self.get_logger().error(

                'Gagal membuka kamera'

            )

            raise RuntimeError(

                'Camera open failed'

            )

        self.get_logger().info(

            'Camera warmup...'

        )

        #

        # Buang frame awal

        #

        for _ in range(10):

            self.cap.read()

        self.get_logger().info(

            'Camera warmup selesai'

        )

        self.frame_counter = 0

        self.last_time = time.time()

        self.saved_debug_frame = False

        #

        # FPS

        #

        self.timer = self.create_timer(

            0.1,    # 10 FPS

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

        #

        # RESIZE

        #

        frame = cv2.resize(

            frame,

            (320, 240)

        )

        mean_pixel = frame.mean()

        self.frame_counter += 1

        now = time.time()

        if now - self.last_time >= 1.0:

            self.get_logger().info(

                f'FPS={self.frame_counter} Mean={mean_pixel:.1f}'

            )

            self.frame_counter = 0

            self.last_time = now

        #

        # SAVE DEBUG

        #

        if not self.saved_debug_frame:

            cv2.imwrite(

                "/tmp/test_ros.jpg",

                frame

            )

            self.saved_debug_frame = True

            self.get_logger().info(

                'Debug image saved: /tmp/test_ros.jpg'

            )

        #

        # ==========================

        # RAW IMAGE

        # ==========================

        #

        raw_msg = self.bridge.cv2_to_imgmsg(

            frame,

            encoding='bgr8'

        )

        raw_msg.header.stamp = (

            self.get_clock().now().to_msg()

        )

        raw_msg.header.frame_id = "camera"

        self.raw_publisher.publish(

            raw_msg

        )

        #

        # ==========================

        # COMPRESSED IMAGE

        # ==========================

        #

        compressed_msg = CompressedImage()

        compressed_msg.header.stamp = (

            self.get_clock().now().to_msg()

        )

        compressed_msg.header.frame_id = "camera"

        compressed_msg.format = "jpeg"

        success, buffer = cv2.imencode(

            '.jpg',

            frame,

            [

                cv2.IMWRITE_JPEG_QUALITY,

                70

            ]

        )

        if success:

            compressed_msg.data = (

                buffer.tobytes()

            )

            self.compressed_publisher.publish(

                compressed_msg

            )

    def destroy_node(self):

        if self.cap.isOpened():

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

        try:

            node.destroy_node()

        except Exception:

            pass

        try:

            rclpy.shutdown()

        except Exception:

            pass

if __name__ == '__main__':

    main()