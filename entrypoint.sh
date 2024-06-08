#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/$USER/ros_ws/install/setup.bash
ros2 launch aip_grasp_planning grasp_planning.launch.py

