#!/bin/bash

trap 'ssh 192.168.0.1 "pkill roslaunch"' INT TERM
ssh jetson@192.168.0.1 "export ROS_MASTER_URI=http://192.168.0.100:11311 && export ROS_IP=192.168.0.1 &&  source /opt/ros/noetic/setup.bash && source /home/jetson/jetson_dual_camera/devel/setup.bash && roslaunch jetson_dual_camera cameras.launch"
wait