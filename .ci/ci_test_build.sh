#!/bin/bash

set -e
echo $GITHUB_WORKSPACE
cp -r $GITHUB_WORKSPACE ~/catkin_ws/src/
cd ~/catkin_ws/src/kinova_gen3_pik_examples
pwd
ls
source /opt/ros/noetic/setup.bash

catkin build --this

echo "Ending build"