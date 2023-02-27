#!/bin/bash

trap 'ssh 192.168.0.1 "pkill roslaunch"' INT TERM
ssh jetson@192.168.0.1 '~/jetson_dual_camera/src/jetson_dual_camera/remote_start_scripts/env_nano.sh'
wait