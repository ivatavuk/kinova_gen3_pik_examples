#!/bin/bash
set -e

sudo apt-get update
sudo apt-get upgrade
#Build and install osqp
cd
git clone --recursive --branch release-0.6.3 https://github.com/osqp/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install

#Install Eigen3
sudo apt install libeigen3-dev

#Build and install osqp-eigen
cd
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build
cd build
cmake ..
make
sudo make install

#Build and install ptsc_eigen
cd 
git clone https://github.com/ivatavuk/ptsc_eigen.git
cd ptsc_eigen
mkdir build
cd build
cmake ..
make
sudo make install

#Install moveit ros-controllers and gazebo
sudo apt-get update
sudo apt-get install -y ros-${ROS_DISTRO}-moveit
sudo apt-get install -y ros-${ROS_DISTRO}-ros-controllers

sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-ros
sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-dev

#Install catkin tools and initialize a catkin ws
sudo apt-get install -y python3-catkin-tools
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init

#install ros_kortex and pik_ros
sudo apt-get install -y python3 python3-pip
python3 -m pip install conan==1.59
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default

cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/Kinovarobotics/ros_kortex.git
git clone -b master https://github.com/ivatavuk/pik_ros.git

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build