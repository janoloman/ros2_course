# ROS2 course

### Summary
ROS1 to ROS2 transition course tips and examples to migrate projects.

## Table of Contents
 - [Overview](#overview)
 - [Setup](#setup)
 - [Ros2](#install-rovtwin)
    - [Create & install pkgs](#create-&-install-pkgs)
    - [Compile and install](#compila-and-install)
    - [Run](#run)
      - [ros-args](#ros-args)
      - [Launch files (ROS2 bringup)](#launch-files-ros2-bringup)
    - [ros1_bridge](#ros1_bridge)
      - [Simple case](#simple-case-run-your-app-with-ros1-and-ros2)
      - [Run with custom msgs and services](#run-with-custom-msgs-and-services)
        - [Compile and install your ROS1 and ROS2 packages](#compile-and-install-your-ros1-and-ros2-packages)
        - [Install ros1_bridge and run](#install-ros1_bridge-and-run)
- [ROS1 to ROS2 migration](#ros1-to-ros2-migration)
- [Troubleshooting](#troubleshooting)
----


## Setup
### Clone the repo
```sh
cd
git clone https://github.com/janoloman/ros2_course
```

### Prerequisites
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
**(Optional) setup environment**
```sh
# Environment setup
echo "# ROS2 foxy" >> ~/.bashrc
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source $HOME/ros2_course/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```


## ROS2
### Create & install pkgs
**Python**
```sh
cd <ros2_ws>

# Create a Python pkg
ros2 pkg create <pkg_name_py> --build-type ament_python --dependencies rclpy std_msgs ...
```
Setup
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
**C++**
```sh
# Create a C++ pkg
ros2 pkg create <pkg_name_cpp> --build-type ament_cmake --dependencies rclcpp std_msgs
```

### Compile and install
```sh
cd <ros2_ws>

# standard build
colcon build 

# build a specific package
colcon build --packages-select ros2_app_py
```

*Python only* (pending): 
If you don't want to build your python pakage every time you modify your files, then
```sh
# build a specific package with symlink
colcon build --packages-select ros2_app_py --symlink-install
```

**Install** 
```sh
# add WS to sourced ROS distro
setup install/setup.bash

# setup only local WS
setup install/local_setup.bash 
```

### Run
```sh
# run a ROS2 node
ros2 run <pkg_name> <node_name>
```
#### ros-args
For renaming nodes, topics, services etc.
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

#### Launch files (ROS2 bringup)
Changed from XML (ROS) to python (ROS2). You can create several launch and configuration files for different node into a single bringup package
```sh
cd <ros2_ws>

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

### ros1_bridge 
#### Simple case
Run your app with ROS1 and ROS2. First terminal: start ROS master
``` sh
source /opt/ros/noetic/setup.bash && roscore
```
second terminal: start ROS2 ros1_bridge
``` sh
source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash
# now you can run ros1_bridge
ros2 run ros1_bridge dynamic_bridge --help
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
**test**
Open 2 more terminals:
```sh
# ROS1 env
source /opt/ros/noetic/setup.bash
rosrun rospy_tutorials talker
```
```sh
# ROS2 env
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp listener
```
#### Run with custom msgs and services
**(Optional) With different or custom messages and services between workspaces**
You need to **define mapping rules between workspaces (ROS1 and ROS2)**.

**messages**
Create a YAML file at your ROS2 package-message or package-service directory
```sh
cd <ros2_ws>/src/<your_ros2_pkg>/<your_msgs_srvs>
touch my_mapping_rules.yaml
```
Add your mapping rules
```yaml
-
# messages mapping
  ros1_package_name: 'my_msgs'
  ros1_message_name: 'CustomString'

  ros2_package_name: 'my_msgs'
  ros2_message_name: 'MyCustomString'

# services mapping
  ros1_package_name: 'my_msgs'
  ros2_package_name: 'my_interfaces'

```

Add your mapping rules to `package.xml`
```xml
...
<export>
    <build_type>ament_cmake</build_type>
    <ros1_bridge mapping_rules="my_mapping_rules.yaml" />
</export>
...
```

Add your mapping rules file to `CMakeLists.txt` as an aditional *install file* after the 'rosidl_generate_interfaces' section 
```cmake
...
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyCustomStrinng.msg"
  "srv/ReseyCounter.srv"
)

install(FILES
  my_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}/
)
...
```
##### Compile and install your ROS1 and ROS2 packages
Open a new terminal, then compile and install your ROS1 package
```sh
# source ROS1 globally
source /opt/ros/noetic/setup.bash

# compile local ROS1 package
cd <ros1_ws>
catkin_make
```
Open a second terminal, then compile and install your ROS2 package 
```sh
# source ROS2 globally
source /opt/ros/foxy/setup.bash

# compile local ROS2 package
cd <ros2_ws>
colcon build
```
##### Install ros1_bridge and run
Open a third terminal, then install ros1_bridge
```sh
# source ROS1 globally
source /opt/ros/noetic/setup.bash
# source ROS2 globally
source /opt/ros/foxy/setup.bash
# source ROS1 ws with custom msg
source <ros1_ws>/devel/setup.bash
# source ROS2 ws with custom msg
source <ros2_ws>/install/setup.bash

# bridge
cd <bridge_ws>
colcon build --packages-select ros1_bridge --cmake-force-configure
# source local ROS2 with ros1_bridge and cutstom msgs
source install/local_setup.bash
# check the custom msg pair
ros2 run ros1_bridge dynamic_bridge --print-pairs

# run
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
Now you can run separadetly your ROS1 and ROS2 programs


## ROS to ROS2 migration
  0) Migrate codebase to ROS1 Noetic

  1) Setup ros1_bridge for the custom interfaces
    - Migrate your Python code to Python3, setting the first line of each script:
```python
#!/usr/bin/env python3
```
    - Setup ros1_bridge

  2) Migrate the nodes
  
  3) Write a launch file for the ROS2 app

  Based on [ROS noetic migration guide](https://wiki.ros.org/noetic/Migration)


## Troubleshooting

- [UserWarning: Usage of dash-separated](https://answers.ros.org/question/386341/ros2-userwarning-usage-of-dash-separated-install-scripts-will-not-be-supported-in-future-versions-please-use-the-underscore-name-install_scripts/)

- [SetuptoolsDeprecationWarning: setup.py install is deprecated](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)
