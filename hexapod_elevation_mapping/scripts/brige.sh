#!/bin/bash
# 自动选择 ROS 2
source /opt/ros/galactic/setup.bash
export ROS_DOMAIN_ID=42
#source /opt/ros/noetic/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics