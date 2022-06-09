# ROS2

## Prerequisites
Install ROS2, based on [ROS2 Foxy install Ubuntu Debian](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
```sh
# setup sources
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop
sudo apt install python3-colcon-common-extensions
```
try some examples
``` sh
# test env
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```
``` sh
# test env
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp listener
```

## Clone the repo
```sh
cd
git clone https://github.com/janoloman/ros2_course
```

## Setup
```sh
# Environment setup
echo "# ROS2 foxy" >> ~/.bashrc
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source $HOME/ros2_course/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

## Create a python pkg
```sh
ros2 pkg create <pkg_name_py> --build-type ament_python --dependencies rclpy std_msgs ...
```
### Install
**Setup**
Add your scripts to the 'setup.py' file inside the 'console_scripts' as follows:
``` python
'console_scripts': [
            "<program_name> = <ros2_package>.<program_file>:<program_function>"
        ],

# example
        'console_scripts': [
            "number_publisher = ros2_app_py.number_publisher:main"
        ],
``` 
Then
```sh
# standard build
colcon build 

# build a specific package
colcon build --packages-select ros2_app_py
```

**(pending)**
If you don't want to build your python pakage every time you modifi your files, then
```sh
# build a specific package with symlink
colcon build --packages-select ros2_app_py --symlink-install
```

## Create a cpp pkg
```sh
ros2 pkg create <pkg_name_cpp> --build-type ament_cmake --dependencies rclcpp std_msgs
```

### Compile and install
```sh
colcon build 
```

## Run

```sh
# run a ROS2 node
ros2 run <pkg_name_cpp> <node_name>
```

**ros-args**
```sh
# rename a node
ros2 run <pkg_name_cpp> <node_name> --ros-args -r __node:=<another_name>

# rename a topic
ros2 run <pkg_name_cpp> <node_name> --ros-args -r __node:=<another_node_name> -r <topic_name>:=<another_topic_name>

# rename a service
ros2 run <pkg_name_cpp> <node_name> --ros-args -r __node:=<another_node_name> -r <service_name>:=<another_service_name>
```

**ROS2 Parameters**
```sh
# run a node and set a parameter
ros2 run <pkg_name_cpp> <node_name> --ros-args -p <parameter_name>:=<parameter_value>
```

You need to declare each parameter at the begining of the constructor class. The type and value of the parameter are set at runtime.

## Launch files
Changed from XML (ROS) to python (ROS2). You can create several launch and configuration files for different node into a single bringup package
```sh
# create a bringup package
ros2 pkg create <name>_bringup
cd <name>_bringup

# remove unused directories
rm -rf include/ && rm -rf src/

# create launch directory
mkdir launch

# create node launch files
touch <node>.launch.py && chmod +x <node>.launch.py

# install
cd <your_ros2_ws> 
colcon build --packages-select <name>_bringup
source ~.bashrc

# run
ros2 launch <name>_bringup <node>.launch.py
```

## ROS to ROS2 migration

### ros1_bridge
first terminal: start ROS master
``` sh
source /opt/ros/noetic/setup.bash
roscore
```
second terminal: start ROS2 ros1_bridge
``` sh
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
# now you can run ros1_bridge
ros2 run ros1_bridge dynamic_bridge --help
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
**test**
open 2 terminals:
```sh
# ROS1 env
rosrun rospy_tutorials talker
```
```sh
# ROS2 env
ros2 run demo_nodes_cpp listener
```
#### custom msgs
you have to define the same message file, with the same fields on both workspaces

**setup**
```sh
# source ROS1 globally
source /opt/ros/noetic/setup.bash
# source ROS2 globally
source /opt/ros/foxy/setup.bash
# source ROS1 ws with custom msg
source <ros1_ws>/devel/setup.bash
# sourceROS2 ws with custom msg
source <ros2_ws>/install/setup.bash


# bridge
cd <bridge_ws>
colcon build --packages-select ros1_bridge --cmake-force-configure
source install/local_setup.bash
# check the custom msg pair
ros2 run ros1_bridge dynamic_bridge --print-pairs
# run
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```


## Troubleshooting

- [UserWarning: Usage of dash-separated](https://answers.ros.org/question/386341/ros2-userwarning-usage-of-dash-separated-install-scripts-will-not-be-supported-in-future-versions-please-use-the-underscore-name-install_scripts/)

- [SetuptoolsDeprecationWarning: setup.py install is deprecated](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)
