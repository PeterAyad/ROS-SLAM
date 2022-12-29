- Each node in ros does a simple fucntion which takes its input as a rostopic and gives its output as a rostopic
- all nodes can be run using "rosrun" or create a single roslaunch file with all arguments

For example:

- summit_xl_gazebo is a package made by summit company to simulate the robot
- the package has a single launch file that takes all the arguments to launch its nodes based on them
- it can launch
	- /rviz node
		- this node draws a map on a gui and can save that map to a file
		- this node listens to the topic /robot/map to take its input
		- it also listens to robot sensors to virualize them as
			- /robot/front_laser/scan 
		- in the gui we can make it listen to more topics
	- /gazebo node
		- this node simulates the robot action and draws the gui
		- it takes inputs as control commands as in
			- /robot/robotnik_base_control/cmd_vel
		- it outputs sensor and odometry as
			- /robot/robotnik_base_control/odom
			- /robot/front_laser/scan
			
	- WE ADD MORE NODES

To visualize all this use rqt
		

