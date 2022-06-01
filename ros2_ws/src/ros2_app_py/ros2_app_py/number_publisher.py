#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class NodePublisherNode(Node):
	def __init__(self):
		super().__init__("number_publisher")
		self.number_ = 4

		self.pub_ = self.create_publisher(Int64, "number", 10)

		self.number_timer_ = self.create_timer(0.5, self.publish_number)
		self.get_logger().info("Number publisher has been started!")

	def print_number(self):
		self.get_logger().info(str(self.number_))

	def publish_number(self):
		msg = Int64()
		msg.data = self.number_
		self.pub_.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = NodePublisherNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()