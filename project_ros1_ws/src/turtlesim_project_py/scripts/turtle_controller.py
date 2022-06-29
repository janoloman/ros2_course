#!/usr/bin/env python3
import rospy
import math
from my_robot_msgs.msg import Coordinates2D
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleController:

    def __init__(self):
        self.pose_ = None
        self.target_ = None
        self.cmd_vel_publisher_ = rospy.Publisher(
            "/turtle1/cmd_vel", Twist, queue_size=10)
        self.pose_subscriber_ = rospy.Subscriber(
            "/turtle1/pose", Pose, self.callback_turtle_pose, queue_size=10)
        self.target_subscriber_ = rospy.Subscriber(
            "target_coordinates", Coordinates2D, self.callback_target)
        self.control_loop_timer_ = rospy.Timer(
            rospy.Duration(0.01), self.control_loop)
        rospy.loginfo("Turtle controller has been started.")

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def callback_target(self, msg):
        if msg.x > 0.0 and msg.x < 11.0 and msg.y > 0.0 and msg.y < 11.0:
            rospy.loginfo("Received new valid target")
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


if __name__ == '__main__':
    rospy.init_node('turtle_controller')
    controller = TurtleController()
    rospy.spin()
