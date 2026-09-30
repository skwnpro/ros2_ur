#!/usr/bin/env python3

import time

import re

import subprocess

import psutil

import rclpy

from rclpy.node import Node

from brushkart_msgs.msg import NetworkStatus

MINI_PC_IP = "192.168.10.1"   # Ubah sesuai IP Ground Station

class NetworkMonitorNode(Node):

    def __init__(self):

        super().__init__("network_monitor_node")

        self.publisher_ = self.create_publisher(

            NetworkStatus,

            "/network_status",

            10

        )

        self.timer = self.create_timer(

            1.0,

            self.timer_callback

        )

        self.last_rx_ms = 0

        net = psutil.net_io_counters()

        self.prev_tx = net.bytes_sent

        self.prev_rx = net.bytes_recv

        self.prev_time = time.time()

        self.get_logger().info("Network Monitor Started")

    def check_ping(self):

        try:

            result = subprocess.run(

                [

                    "ping",

                    "-c", "4",

                    "-W", "1",

                    MINI_PC_IP

                ],

                capture_output=True,

                text=True

            )

            output = result.stdout

            loss_match = re.search(

                r'(\d+(?:\.\d+)?)% packet loss',

                output

            )

            latency_match = re.search(

                r'=\s*[\d.]+/([\d.]+)/',

                output

            )

            packet_loss = (

                float(loss_match.group(1))

                if loss_match else 100.0

            )

            latency = (

                float(latency_match.group(1))

                if latency_match else -1.0

            )

            connected = packet_loss < 100.0

            if connected:

                self.last_rx_ms = 0

            else:

                self.last_rx_ms += 1000

            return connected, latency, packet_loss

        except Exception as e:

            self.get_logger().warning(

                f"Ping Error: {str(e)}"

            )

            self.last_rx_ms += 1000

            return False, -1.0, 100.0

    def get_bandwidth(self):

        current_time = time.time()

        net = psutil.net_io_counters()

        tx_bytes = net.bytes_sent

        rx_bytes = net.bytes_recv

        dt = current_time - self.prev_time

        if dt <= 0:

            return 0.0, 0.0

        tx_kbps = (

            (tx_bytes - self.prev_tx) * 8

            / 1024

            / dt

        )

        rx_kbps = (

            (rx_bytes - self.prev_rx) * 8

            / 1024

            / dt

        )

        self.prev_tx = tx_bytes

        self.prev_rx = rx_bytes

        self.prev_time = current_time

        return tx_kbps, rx_kbps

    def get_system_usage(self):

        cpu_usage = psutil.cpu_percent()

        ram_usage = psutil.virtual_memory().percent

        return cpu_usage, ram_usage

    def timer_callback(self):

        connected, latency, packet_loss = self.check_ping()

        tx_kbps, rx_kbps = self.get_bandwidth()

        cpu_usage, ram_usage = self.get_system_usage()

        msg = NetworkStatus()

        msg.connected = connected

        msg.latency_ms = float(latency)

        msg.packet_loss_percent = float(packet_loss)

        msg.last_rx_ms = int(self.last_rx_ms)

        msg.tx_kbps = float(tx_kbps)

        msg.rx_kbps = float(rx_kbps)

        msg.cpu_usage = float(cpu_usage)

        msg.ram_usage = float(ram_usage)

        self.publisher_.publish(msg)

        self.get_logger().info(

            f"Conn={msg.connected} "

            f"Lat={msg.latency_ms:.1f}ms "

            f"Loss={msg.packet_loss_percent:.1f}% "

            f"TX={msg.tx_kbps:.1f}kbps "

            f"RX={msg.rx_kbps:.1f}kbps "

            f"CPU={msg.cpu_usage:.1f}% "

            f"RAM={msg.ram_usage:.1f}%"

        )

def main(args=None):

    rclpy.init(args=args)

    node = NetworkMonitorNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":

    main()