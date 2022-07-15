from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	ld = LaunchDescription()

	turtlesim_node = Node(
		package="turtlesim",
		executable="turtlesim_node"
	)

	turtle_controller_node = Node(
		package="turtlesim_project_py",
		executable="turtle_controller"
	)

	turtle_publisher_node = Node(
		package="turtlesim_project_py",
		executable="target_publisher"
	)

	ld.add_action(turtlesim_node)
	ld.add_action(turtle_controller_node)
	ld.add_action(turtle_publisher_node)

	return ld