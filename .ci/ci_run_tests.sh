#!/bin/bash
set -e

source ~/catkin_ws/devel/setup.bash

#install tmux
sudo apt-get install -y tmux

tmux new-session -d -s tests_session
tmux split-window -h -t tests_session
tmux send-keys -t tests_session:0.0 'roslaunch kinova_gen3_pik_examples spawn_kortex_robot.launch gazebo_gui:=false start_rviz:=false' Enter
sleep 20

cd ~/catkin_ws/build/kinova_gen3_pik_examples
ctest
echo "Ending build"