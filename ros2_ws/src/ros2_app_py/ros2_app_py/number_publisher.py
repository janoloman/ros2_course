#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class NodePublisherNode(Node):
	def __init__(self):
		super().__init__("number_publisher")
		self.declare_parameter("number_to_publish", 2)
		self.declare_parameter("number_publish_frecuency", 1)

		self.number_ = self.get_parameter("number_to_publish").value
		self.publish_frecuency_ = self.get_parameter("number_publish_frecuency").value

		self.pub_ = self.create_publisher(Int64, "number", 10)

		self.number_timer_ = self.create_timer(1/self.publish_frecuency_, self.publish_number)
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