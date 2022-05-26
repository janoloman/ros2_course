#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main(args=None):
	rclpy.init(args=args)
	node = Node('number_publisher')
	logger=node.get_logger()
	# node.get_logger().info('Hello ROS2')
	logger.info('Hello ROS2')
	rclpy.shutdown()

if __name__ == '__main__':
	main()