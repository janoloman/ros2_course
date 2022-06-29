cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_project_py)

find_package(catkin REQUIRED COMPONENTS
  my_robot_msgs
  rospy
  turtlesim
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS my_robot_msgs rospy turtlesim geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
