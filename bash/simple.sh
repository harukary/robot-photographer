#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/harukary/catkin_ws/devel/setup.bash

echo "Launching application, please wait!"
roslaunch turtlebot3_gazebo turtlebot3_house.launch