#!/bin/bash

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

mkdir -p ~/turtlebot_sim/src
cd ~/turtlebot_sim/src

git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b humble-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b humble-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b humble-devel
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble-devel

cd ~/turtlebot_sim
colcon build 

source install/setup.bash

ros2 launch turtlebot3_gazebo empty_world.launch.py


