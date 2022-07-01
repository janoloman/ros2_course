#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from my_robot_interfaces.msg import Coordinates2D


class TargetPublisherNode(Node):
    def __init__(self):
        super().__init__("target_publisher")
        self.declare_parameter("publish_interval", 3.0)
        publish_interval = self.get_parameter("publish_interval").value

        self.pub_ = self.create_publisher(
            Coordinates2D, "target_coordinates", 10)
        self.timer_ =self.create_timer(
            publish_interval, self.send_random_coordinates)
        self.get_logger().info("Target publisher has been started")

    def publish_coordinates(self, x, y):
        msg = Coordinates2D()
        msg.x = x
        msg.y = y
        self.pub_.publish(msg)
        
    def send_random_coordinates(self):
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        self.publish_coordinates(x, y)

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
