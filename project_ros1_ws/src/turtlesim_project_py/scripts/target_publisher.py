#!/usr/bin/env python3
import rospy
import random
from my_robot_msgs.msg import Coordinates2D


class TargetPublisher:

    def __init__(self):
        publish_interval = rospy.get_param("~publish_interval", 3.0)
        self.pub_ = rospy.Publisher(
            "target_coordinates", Coordinates2D, queue_size=10)
        self.timer_ = rospy.Timer(rospy.Duration(
            publish_interval), self.send_random_coordinates)
        rospy.loginfo("Target publisher has been started")

    def publish_coordinates(self, x, y):
        msg = Coordinates2D()
        msg.x = x
        msg.y = y
        self.pub_.publish(msg)

    def send_random_coordinates(self, event=None):
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        self.publish_coordinates(x, y)


if __name__ == '__main__':
    rospy.init_node('target_publisher')
    coordinates_generator = TargetPublisher()
    rospy.spin()
