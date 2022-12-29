#!/usr/bin/env python3

# ROS libraries
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64
import os
from getkey import getkey, keys

import numpy

# Pre-defined global variables

_qwe = Vector3()
_asd = Vector3()
_pose = Vector3()
_cmd_vel = Twist()
_cam_pan = Float64()
_cam_tilt = Float64()
linear_vel = 0.0
angular_vel = 0.0
wasLastBack = 0
wasLastFront = 0

##############################################################
##############################################################
# Practical session P1.1
# The information retrived from the interface is stored in the
# variables _qwe, _asd, _pose
# Your task is produce the proper velocity output for the robot
# and the set the position of the camera.
# The velocity must be set in the variable _cmd_vel,
# the pan of the camera in the variable _cam_pan and the
# tilt of the camera in the variable _cam_tilt.

# If you require new global variables, add them here
# and declare them in the function process_data()


# Insert your code inside the function process_data
def process_data():
    # You can acces the value of the following variables:
    # If pressed key (q) _qwe.x = 1.0 ; (w) _qwe.y = 1.0; (e) _qwe.z = 1.0. Otherwise, they all equal to 0.0
    # If pressed key (a) _asd.x = 1.0 ; (s) _asd.y = 1.0; (d) _asd.z = 1.0. Otherwise, they all equal to 0.0
    # If click in the interface, _pose.x = vertical coordinate ;  _pose.y = horizontal coordinate. Otherwise, they all equal to -1.0
    # If click out of the interface, or the mouse leaves the interface, _pose.x = _pose.y = -1.0

    global _qwe, _asd, _pose, _cmd_vel, _cam_pan, _cam_tilt, linear_vel, angular_vel, wasLastBack, wasLastFront

    if _qwe.y == 1.0:
        linear_vel += 1.0
    elif _asd.y == 1.0:
        linear_vel += -1.0
        
    if _asd.x == 1.0:
        angular_vel += 1.0
    elif _asd.z == 1.0:
        angular_vel += -1.0
    else:
        angular_vel = 0.0
        
    if _pose.x != -1.0:
        _cam_tilt.data = _pose.x
        _cam_pan.data = _pose.y
        
        
    if wasLastBack == False and _asd.y == 1.0:
        linear_vel = 0.0
        angular_vel = 0.0
        wasLastBack = True
    elif _asd.y == 1.0:
        wasLastFront = False
    
    if wasLastFront == False and _qwe.y == 1.0:
        linear_vel = 0.0
        angular_vel = 0.0
        wasLastFront = True
    elif _qwe.y == 1.0:
        wasLastBack = False


    # Outputs: replace the value 0.0 with your output
    # linear velocity, [-3.0,3.0] (+-1.5 m/s)
    _cmd_vel.linear.x = linear_vel
    # angular velocity, [-6.0,6.0] (+-3.0 rad/s)
    _cmd_vel.angular.z = angular_vel
    # # camera pan, [-1.57,1.57] (rad)
    # _cam_pan.data = 0.0
    # # camera tilt, [-1.57,1.57] (rad)
    # _cam_tilt.data = 0.0

    # Cleaning keyboard input data after being processed, if you want
    # to keep it, you must store it in a global variable.
    # Other variables are not cleaded here. 
    clean_keyboard_input()

##############################################################
##############################################################

# Cleans data after being processed
def clean_keyboard_input():
    global _qwe, _asd
    _qwe.x = _qwe.y = _qwe.z = 0
    _asd.x = _asd.y = _asd.z = 0

# Define callbacks and functions to process incoming messages
def qwe_callback(msg):
    global _qwe
    # rospy.loginfo("Received message: %s", msg)
    _qwe = msg

def asd_callback(msg):
    global _asd
    # rospy.loginfo("Received message: %s", msg)
    _asd = msg

def pose_callback(msg):
    global _pose
    # rospy.loginfo("Received message: %s", msg)
    _pose = msg

# Main

if __name__ == '__main__':

    # Add here the name of the ROS node. In ROS, node names must be unique.
    rospy.init_node('robot_teleoperation_node')

    # Publish messages
    pub_vel = rospy.Publisher('/robot/cmd_vel/', Twist, queue_size=10)
    pub_pan = rospy.Publisher('/robot/joint_pan_position_controller/command', Float64, queue_size = 1)
    pub_tilt = rospy.Publisher('/robot/joint_tilt_position_controller/command', Float64, queue_size = 1)

    rate=rospy.Rate(30)

    while not rospy.is_shutdown():
        
        x = getkey(blocking=True)
        
        if x == 'w':
            _qwe = Vector3(0,1,0)
        elif x == 's':
            _asd = Vector3(0,1,0)
        elif x == 'a':
            _asd = Vector3(1,0,0)
        elif x == 'd':
            _asd = Vector3(0,0,1)
            

        process_data()
        rospy.loginfo("Sending message: %s", _cmd_vel)
        pub_vel.publish(_cmd_vel)
        pub_pan.publish(_cam_pan)
        pub_tilt.publish(_cam_tilt)
        rate.sleep()
