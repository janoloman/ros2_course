cmake_minimum_required(VERSION 3.0.2)
project(my_robot_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  Coordinates2D.msg
)

generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_robot_msgs
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

