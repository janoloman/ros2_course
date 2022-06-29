from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	ld = LaunchDescription()

	remap_number_topic = ("number", "my_number")

	number_publisher_node = Node(
		# package="ros2_app_py",
		package="ros2_app_cpp",
		executable="number_publisher",
		name="my_number_publisher", # rename the node
		remappings=[								# rename topics, services, etc.
			remap_number_topic
		],
		parameters=[ # array of dictionraries
			{"number_to_publish":4},
			{"number_publish_frecuency":5.0},
		]
	)
	number_counter_node = Node(
		package="ros2_app_py",
		# package="ros2_app_cpp",
		executable="number_counter",
		name="my_number_counter",
		remappings=[
			remap_number_topic,
			("reset_counter", "my_reset_counter")
		]
	)

	ld.add_action(number_publisher_node)
	ld.add_action(number_counter_node)

	return ld