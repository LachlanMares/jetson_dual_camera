# Running with Docker

Follow these instructions if you would like to run this system using Docker.

## Installing Docker
See the [Docker](https://www.docker.com) website.

## Clone the Repository
Can be cloned anywhere.

## Ethernet Connection
Turn on the Jetson Nano and connect an ethernet cable to it coming from the host computer.

On the host computer, change the ethernet address to `192.168.0.100`.  

This can be done on Ubuntu by:
1. Bringing up the ethernet settings
2. Clicking the gear icon
3. Disabling IPv6
4. Setting IPv4 to manual and entering the IP address with a mask of `255.255.255.0`
5. Clicking `Apply`

If all went successfully, you should now be able to ping the Jetson Nano:
```bash
ping 192.168.0.1
```

If it doesn't work you may need to remove and re-connect the ethernet cable.

## Building the Image
Navigate into the `jetson_dual_camera` directory and execute the following command:
```bash
make build
```

This only needs to be done the first time.

## Entering the Container
Start the container:
```bash
make up
```

Enter the container:
```bash
make exec
```

## SSH
An initial connection needs to be established with the Jetson Nano. Type `yes` when prompted for your fingerprint.
```bash
ssh jetson@192.168.0.1
```
Password: `jetson`  

Once connected, disconnect from the jetson by typing `exit`.

## Building the Code
Compile the code:
```bash
catkin_make
```

## Running the Code
1. Setup the ros workspace (must be done for each new bash session):
```bash
jetson_dual_camera_remote
```

2. Run the cameras on the Jetson Nano:
```bash
roslaunch jetson_dual_camera remote_cameras.launch 
```

3. Open a new bash shell and enter the following commands:
```bash
make exec
jetson_dual_camera_remote
```

4. In the new bash shell, record the camera feed:
```bash
roslaunch jetson_dual_camera video_creator.launch
```

To stop recording the camera feed, type `Ctrl+C`. This will result in the videos being saved in the `videos/` directory. Note that old videos will be overwritten.

## Cleanup

To stop the Jetson Nano cameras, type `Ctrl+C` in the first bash shell.

To exit the container, type enter `exit` in both bash shells.

At this point you can choose to enter the container again by using `make exec` and start following the instructions from [Running the Code](#running-the-code).

You can kill (delete) the container by running `make down`. Once you do this you will need to follow the instructions from [Entering the Container](#entering-the-container). Restarting the host computer will also have this effect.


# Running Locally

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