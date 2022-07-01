#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from my_robot_interfaces.msg import Coordinates2D
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_ = None
        self.target_ = None
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_turtle_pose, 10)
        self.target_subscriber_ = self.create_subscription(
            Coordinates2D, "target_coordinates", self.callback_target, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("Turtle controller has been started")

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def callback_target(self, msg):
        if msg.x > 0.0 and msg.x < 11.0 and msg.y > 0.0 and msg.y < 11.0:
            self.get_logger().info("Received new valid target")
            self.target_ = msg
            
    def control_loop(self, event=None):
        if self.pose_ == None or self.target_ == None:
            return

        dist_x = self.target_.x - self.pose_.x
        dist_y = self.target_.y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        msg = Twist()

        if distance > 0.5:
            # position
            msg.linear.x = 2*distance

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            msg.angular.z = 6*diff
        else:
            # target reached!
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.target_ = None

        self.cmd_vel_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
