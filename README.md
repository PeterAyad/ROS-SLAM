# To clone this repo
```bash
git clone --recursive git@github.com:PeterAyad/ROS-SLAM-Project.git
```

# To run 
```bash
sed -i -e  '104d;105d' src/summit_xl_sim/summit_xl_gazebo/launch/summit_xl_one_robot.launch
catkin_make
roscore
source devel/setup.bash 
roslaunch launch_pkg mapping.launch

# or

roslaunch launch_pkg slam.launch

# and 

rosrun teleoperation teleoperation.py 
```
