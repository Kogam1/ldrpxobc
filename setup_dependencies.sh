#!/bin/bash
# setup_dependencies.sh

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble (Ubuntu 22.04 compatible with 24.04)
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep2
sudo rosdep init
rosdep update

# Install required packages
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libpcap-dev \
    libeigen3-dev \
    libboost-all-dev \
    python3-pip \
    wget

# Install Python packages
pip3 install \
    numpy \
    open3d \
    transforms3d \
    pymavlink \
    pyserial

# Install ROS packages
sudo apt install -y \
    ros-humble-velodyne \
    ros-humble-robot-localization \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs

echo "Dependencies installed successfully!"
