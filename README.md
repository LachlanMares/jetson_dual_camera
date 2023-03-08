# jetson_dual_camera



## ROS Stuff
Install ROS Noetic if you haven't already, requires Ubuntu 20.04
  
[ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

clone this repo into home directory

https://github.com/LachlanMares/jetson_dual_camera

open a terminal and navigate to repo:

```bash
cd ~/jetson_dual_camera
```

source ROS:

```bash
source /opt/ros/noetic/setup.bash
```

Make catkin workspace:

```bash
catkin_make
```

After building you can add this function to your .bashrc file. 

function jetson_dual_camera_remote()
{
  source /opt/ros/noetic/setup.bash
  source ~/jetson_dual_camera/devel/setup.bash
  export ROS_IP="192.168.0.100"
}

To get to .bashrc type in terminal:

```bash
cd 
sudo gedit .bashrc
```

## Network Stuff
Jetson has a fixed IP of 192.168.0.1, username: jetson, password: jetson
Set your ip to 192.168.0.100

You will need to create ssh keys and add the public key to the jetson's known_hosts file 

https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys-on-ubuntu-20-04

## Running Stuff
Starting the jetson cameras:

You will need to source ROS, open new terminal and type in:

```bash
jetson_dual_camera_remote
```

Assuming the Jetson is on type into terminal:
```bash
roslaunch jetson_dual_camera remote_cameras.launch 
```

Check for images, open a new terminal and type in:

```bash
jetson_dual_camera_remote
roslaunch jetson_dual_camera rviz.launch
```

Rviz should appear and the two camera feeds should be displayed. Close Rviz before trying to record stuff

To record videos:

```bash
jetson_dual_camera_remote
roslaunch jetson_dual_camera video_creator.launch
```

This will start recording 2 videos into /videos directory. Video names along with other properties are defined in the /config/camera_x_parameters.yaml. Close terminal with ctrl-C to stop recording