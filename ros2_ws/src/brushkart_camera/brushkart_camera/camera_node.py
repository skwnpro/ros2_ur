#!/usr/bin/env python3

import cv2

import time

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

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

            CompressedImage,

            '/camera/compressed',

            qos

        )

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

            0.05,   # 20 FPS

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

        # Resize image

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

        # Simpan 1 frame debug

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

        # JPEG Encode

        #

        success, buffer = cv2.imencode(

            '.jpg',

            frame,

            [

                cv2.IMWRITE_JPEG_QUALITY,

                50

            ]

        )

        if not success:

            self.get_logger().warning(

                'JPEG encode gagal'

            )

            return

        msg = CompressedImage()

        msg.header.stamp = (

            self.get_clock().now().to_msg()

        )

        msg.header.frame_id = "camera"

        msg.format = "jpeg"

        msg.data = buffer.tobytes()

        self.publisher_.publish(msg)

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