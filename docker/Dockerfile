# Ubuntu Focal with ROS Noetic
FROM ros:noetic-ros-core-focal

RUN apt-get update

# Setup script sourcing every time a new shell is launched.
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# For 
RUN printf "function jetson_dual_camera_remote() {\n\
  source /jetson_dual_camera/devel/setup.bash\n\
  export ROS_IP=\"192.168.0.100\"\n\
}" >> ~/.bashrc

# Install rosdep
RUN apt-get -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Install Pip, Nano, and ping
RUN apt-get -y install python3-pip nano iputils-ping

# Install necessary ROS packages
RUN apt-get update
RUN apt-get -y install ros-noetic-rviz
RUN apt-get update
RUN apt-get -y install ros-noetic-cv-bridge
RUN apt-get -y install ros-noetic-vision-opencv

# Install Catkin tools
# RUN apt-get -y install wget
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get -y install python3-catkin-tools

# Other
RUN apt-get -y install make
RUN apt-get -y install libudev-dev

# Initialise rosdep
RUN rosdep init
RUN rosdep update

# Create workspace
RUN mkdir -p ~/catkin_ws/src

# CMD ["bash"]
ENTRYPOINT bash