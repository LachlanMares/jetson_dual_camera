#!/bin/bash

export ROS_MASTER_URI=http://192.168.0.1:11311
export ROS_IP=192.168.0.1
source /opt/ros/noetic/setup.bash
source /home/jetson/jetson_dual_camera/devel/setup.bash

nohup roslaunch jetson_dual_camera cameras.launch