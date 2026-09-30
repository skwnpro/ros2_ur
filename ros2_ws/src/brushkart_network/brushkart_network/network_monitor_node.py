import rclpy

from rclpy.node import Node

from brushkart_msgs.msg import NetworkStatus

class NetworkMonitorNode(Node):

    def __init__(self):

        super().__init__('network_monitor_node')

        self.publisher_ = self.create_publisher(

            NetworkStatus,

            '/network_status',

            10

        )

        self.timer = self.create_timer(

            1.0,

            self.timer_callback

        )

        self.get_logger().info(

            "Network Monitor Started"

        )

    def timer_callback(self):

        msg = NetworkStatus()

        msg.connected = True

        msg.latency_ms = 5.0

        msg.packet_loss_percent = 0.0

        msg.last_rx_ms = 0

        msg.tx_kbps = 1000.0

        msg.rx_kbps = 2000.0

        msg.cpu_usage = 20.0

        msg.ram_usage = 35.0

        self.publisher_.publish(msg)

        self.get_logger().info(

            f"Latency={msg.latency_ms}"

        )

def main(args=None):

    rclpy.init(args=args)

    node = NetworkMonitorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()