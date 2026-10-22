#!/usr/bin/env python3
"""camera_node — kamera underwater robot (CSI IMX290 atau USB V4L2).

Satu file ini dipakai untuk SEMUA kamera; bedakan tiap kamera lewat
namespace + parameter saat launch:

  * Kamera utama IMX290 (CSI) berjalan di balik libcamerify (emulasi
    V4L2 di atas libcamera) — prosesnya harus dibungkus libcamerify,
    diatur dari launch file (prefix). Lihat catatan perang kamera
    26-28 September 2026 di vault sebelum mengubah jalur ini.
  * Kamera USB adalah V4L2 native — JANGAN dibungkus libcamerify,
    jalankan sebagai instance terpisah dengan device_path-nya sendiri
    (pakai /dev/v4l/by-id/... agar tidak tertukar setelah reboot).

Topic (relatif terhadap namespace node):
  * image_raw   (sensor_msgs/Image)           — hanya bila publish_raw
  * compressed  (sensor_msgs/CompressedImage) — stream utama ke GS
  * camera_info (sensor_msgs/CameraInfo)      — info dasar (estimasi)

Keputusan desain:
  * Pengambilan frame berjalan di thread sendiri dan hanya frame
    TERBARU yang diterbitkan — frame lama dibuang, bukan diantrekan
    (pelajaran dari kasus lag/freeze di RViz, Sept 2026).
  * QoS BEST_EFFORT + antrean 1: untuk video langsung, frame telat
    tidak berguna. (QoS RELIABLE versi lama dipilih hanya untuk
    menghilangkan warning RViz — bukan kebutuhan sistem.)
  * Encode JPEG dilewati bila tidak ada subscriber, demi CPU Raspi.
  * Bila kamera gagal dibaca beruntun, node mencoba membuka ulang
    sendiri (reconnect) alih-alih diam selamanya.
"""

import threading
import time

import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


JUMLAH_GAGAL_RECONNECT = 10   # gagal baca beruntun sebelum reconnect
JEDA_RECONNECT_S = 1.0


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # --- Parameter (per instance, diatur dari launch file) ---
        self.declare_parameter('device_id', 0)
        self.declare_parameter('device_path', '')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('frame_id', 'camera')

        self.device_id = self.get_parameter('device_id').value
        self.device_path = self.get_parameter('device_path').value
        self.req_width = self.get_parameter('width').value
        self.req_height = self.get_parameter('height').value
        self.fps = float(self.get_parameter('fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.publish_raw = self.get_parameter('publish_raw').value
        self.frame_id = self.get_parameter('frame_id').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.raw_pub = self.create_publisher(Image, 'image_raw', qos)
        self.compressed_pub = self.create_publisher(
            CompressedImage, 'compressed', qos)
        self.info_pub = self.create_publisher(
            CameraInfo, 'camera_info', qos)

        self.bridge = CvBridge()

        # Slot frame terbaru, dibagi dengan thread capture.
        self._lock = threading.Lock()
        self._latest_frame = None
        self._frame_counter = 0
        self._published_counter = 0

        self._stop_event = threading.Event()
        self._cap = None
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.timer = self.create_timer(1.0 / self.fps, self.publish_image)

        self.get_logger().info(
            f'Camera Node Started (device='
            f'{self.device_path or self.device_id}, '
            f'minta {self.req_width}x{self.req_height}@{self.fps:.0f})')

    # ------------------------------------------------------------------
    # Thread capture
    # ------------------------------------------------------------------

    def _open_capture(self):
        """Buka kamera dan minta format; kembalikan True bila berhasil."""
        sumber = self.device_path if self.device_path else self.device_id
        cap = cv2.VideoCapture(sumber, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.req_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Laporkan format yang BENAR-BENAR dinegosiasikan — yang
        # diberikan pipeline belum tentu sama dengan yang diminta.
        aktual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        aktual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        aktual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Kamera terbuka: {aktual_w}x{aktual_h}@{aktual_fps:.0f} '
            f'(diminta {self.req_width}x{self.req_height}@{self.fps:.0f})')

        # Buang frame awal selagi auto-exposure menyesuaikan.
        for _ in range(5):
            cap.read()

        self._cap = cap
        return True

    def _capture_loop(self):
        gagal_beruntun = 0
        while not self._stop_event.is_set():
            if self._cap is None:
                if not self._open_capture():
                    self.get_logger().warning(
                        'Kamera belum bisa dibuka, mencoba lagi...',
                        throttle_duration_sec=10.0)
                    self._stop_event.wait(JEDA_RECONNECT_S)
                continue

            ok, frame = self._cap.read()
            if not ok or frame is None:
                gagal_beruntun += 1
                if gagal_beruntun >= JUMLAH_GAGAL_RECONNECT:
                    self.get_logger().warning(
                        f'{gagal_beruntun}x gagal membaca kamera — '
                        'membuka ulang (reconnect)')
                    self._cap.release()
                    self._cap = None
                    gagal_beruntun = 0
                    self._stop_event.wait(JEDA_RECONNECT_S)
                continue

            gagal_beruntun = 0
            with self._lock:
                self._latest_frame = frame
                self._frame_counter += 1

    # ------------------------------------------------------------------
    # Publikasi
    # ------------------------------------------------------------------

    def _buat_camera_info(self, width, height, stamp):
        """CameraInfo dasar. Focal length masih ESTIMASI (selebar
        frame) — cukup agar penampil standar bisa bekerja, tapi belum
        hasil kalibrasi; ganti dengan nilai kalibrasi sesungguhnya
        bila suatu hari dipakai untuk pengukuran."""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        f = float(width)
        cx = width / 2.0
        cy = height / 2.0
        info.k = [f, 0.0, cx,
                  0.0, f, cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0,
                  0.0, f, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def publish_image(self):
        with self._lock:
            frame = self._latest_frame
            counter = self._frame_counter

        # Tidak ada frame baru sejak publikasi terakhir -> lewati.
        if frame is None or counter == self._published_counter:
            return
        self._published_counter = counter

        stamp = self.get_clock().now().to_msg()
        height, width = frame.shape[:2]

        mau_raw = self.publish_raw \
            and self.raw_pub.get_subscription_count() > 0
        mau_compressed = self.compressed_pub.get_subscription_count() > 0
        mau_info = self.info_pub.get_subscription_count() > 0

        if mau_raw:
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = self.frame_id
            self.raw_pub.publish(raw_msg)

        if mau_compressed:
            sukses, buffer = cv2.imencode(
                '.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if sukses:
                comp_msg = CompressedImage()
                comp_msg.header.stamp = stamp
                comp_msg.header.frame_id = self.frame_id
                comp_msg.format = 'jpeg'
                comp_msg.data = buffer.tobytes()
                self.compressed_pub.publish(comp_msg)

        if mau_info and (mau_raw or mau_compressed):
            self.info_pub.publish(
                self._buat_camera_info(width, height, stamp))

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._stop_event.set()
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()
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
