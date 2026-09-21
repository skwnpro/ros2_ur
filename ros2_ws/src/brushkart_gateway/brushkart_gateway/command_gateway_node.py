#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class CommandGatewayNode(Node):
	def __init__(self):
		super().__init__('command_gateway_node')
		self.get_logger().info(
			'BrushKart Gateway Started'
		)

def main(args=None):
	rclpy.init(args=args)
	node = CommandGatewayNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
