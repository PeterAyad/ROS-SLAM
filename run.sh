#!/bin/bash
sed -i -e  '104d;105d' src/summit_xl_sim/summit_xl_gazebo/launch/summit_xl_one_robot.launch
catkin_make
roscore &
source devel/setup.bash 
roslaunch launch_pkg all.launch

source devel/setup.bash; roslaunch mapping mapping_node.py
source devel/setup.bash & roslaunch teleoperation teleoperation_node.py