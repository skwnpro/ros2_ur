#!/usr/bin/env python3
"""network_monitor_node — pemantau kesehatan tether dari sisi robot.

Berjalan di underwater robot. Setiap detik:
  * ping ke ground station (parameter `target_ip`),
  * mengukur bandwidth, CPU, dan RAM Raspberry Pi,
lalu menerbitkan hasilnya sebagai /network_status
(brushkart_msgs/NetworkStatus).

Catatan arah pengukuran: node ini mengukur kesehatan arah
robot -> ground station. Kesehatan arah sebaliknya (perintah
GS -> robot) tidak diukur di sini; perlindungannya ada pada
heartbeat + watchdog di rantai perintah (command_gateway_node
dan relay_operator_node).

`last_rx_ms` = milidetik sejak ping terakhir yang BERHASIL
(0 selama koneksi sehat).
"""

import re
import subprocess
import time

import psutil

import rclpy
from rclpy.node import Node

from brushkart_msgs.msg import NetworkStatus


class NetworkMonitorNode(Node):

    def __init__(self):
        super().__init__('network_monitor_node')

        # IP ground station yang di-ping.
        self.declare_parameter('target_ip', '192.168.10.1')
        self.target_ip = self.get_parameter('target_ip').value

        self.publisher_ = self.create_publisher(
            NetworkStatus, '/network_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Waktu ping terakhir yang berhasil (jam dinding, detik).
        self.last_success_time = time.time()

        net = psutil.net_io_counters()
        self.prev_tx = net.bytes_sent
        self.prev_rx = net.bytes_recv
        self.prev_time = time.time()

        self.get_logger().info(
            f'Network Monitor Started (target {self.target_ip})')

    # ------------------------------------------------------------------
    # Pengukuran
    # ------------------------------------------------------------------

    def check_ping(self):
        """Ping singkat ke ground station.

        2 paket dengan timeout 1 detik per paket: cukup untuk menilai
        koneksi tanpa memblokir timer terlalu lama (versi lama 4 paket
        bisa menahan callback sampai ~4 detik saat koneksi buruk).
        """
        try:
            result = subprocess.run(
                ['ping', '-c', '2', '-W', '1', self.target_ip],
                capture_output=True, text=True)

            output = result.stdout

            loss_match = re.search(
                r'(\d+(?:\.\d+)?)% packet loss', output)
            latency_match = re.search(
                r'=\s*[\d.]+/([\d.]+)/', output)

            packet_loss = (
                float(loss_match.group(1)) if loss_match else 100.0)
            latency = (
                float(latency_match.group(1)) if latency_match else -1.0)
            connected = packet_loss < 100.0

            return connected, latency, packet_loss
        except Exception as e:
            self.get_logger().warning(f'Ping error: {e}')
            return False, -1.0, 100.0

    def get_bandwidth(self):
        current_time = time.time()
        net = psutil.net_io_counters()
        tx_bytes = net.bytes_sent
        rx_bytes = net.bytes_recv

        dt = current_time - self.prev_time
        if dt <= 0:
            return 0.0, 0.0

        tx_kbps = (tx_bytes - self.prev_tx) * 8 / 1024 / dt
        rx_kbps = (rx_bytes - self.prev_rx) * 8 / 1024 / dt

        self.prev_tx = tx_bytes
        self.prev_rx = rx_bytes
        self.prev_time = current_time

        return tx_kbps, rx_kbps

    def get_system_usage(self):
        return psutil.cpu_percent(), psutil.virtual_memory().percent

    # ------------------------------------------------------------------
    # Siklus publikasi
    # ------------------------------------------------------------------

    def timer_callback(self):
        connected, latency, packet_loss = self.check_ping()
        tx_kbps, rx_kbps = self.get_bandwidth()
        cpu_usage, ram_usage = self.get_system_usage()

        sekarang = time.time()
        if connected:
            self.last_success_time = sekarang
            last_rx_ms = 0
        else:
            last_rx_ms = int((sekarang - self.last_success_time) * 1000)

        msg = NetworkStatus()
        msg.connected = connected
        msg.latency_ms = float(latency)
        msg.packet_loss_percent = float(packet_loss)
        msg.last_rx_ms = last_rx_ms
        msg.tx_kbps = float(tx_kbps)
        msg.rx_kbps = float(rx_kbps)
        msg.cpu_usage = float(cpu_usage)
        msg.ram_usage = float(ram_usage)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NetworkMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
