#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NodePublisherNode(Node):
	def __init__(self):
		super().__init__("number_publisher")
		self.numer_ = 4
		self.numer_timer_ = self.create_timer(0.5, self.print_number)
		self.get_logger().info("Number publisher has been started!")

	def print_number(self):
		self.get_logger().info(str(self.numer_))

def main(args=None):
	rclpy.init(args=args)
	node = NodePublisherNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()