cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_project_cpp)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  turtlesim
  my_robot_msgs
)

catkin_package(
   CATKIN_DEPENDS geometry_msgs roscpp turtlesim my_robot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(target_publisher src/target_publisher.cpp)
add_dependencies(target_publisher my_robot_msgs_generate_messages_cpp)
target_link_libraries(target_publisher ${catkin_LIBRARIES})

add_executable(turtle_controller src/turtle_controller.cpp)
add_dependencies(turtle_controller my_robot_msgs_generate_messages_cpp)
target_link_libraries(turtle_controller ${catkin_LIBRARIES})
