#!/usr/bin/env python3

# ROS libraries
import termios
import tty
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64
import select , sys


FORWARD = 'w'
BACKWARD = 's'
LEFT = 'a'
RIGHT = 'd'

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getKey():
    if not isData():
        return None
    else:
        return sys.stdin.read(1) # read 2 characters
def clamp(x,x_min,x_max):
    return max(x_min, min(x_max, x))
class Robot:
    angular_acceleration = 0
    angular_velocity = 0
    max_angular_velocity = 0

    acceleration = 0
    velocity = 0
    max_velocity = 0

    
    def __init__(self,acceleration=10,angular_acceleration=10,max_velocity=5,max_angular_velocity=3) -> None:
        self.max_velocity         = max_velocity
        self.max_angular_velocity = max_angular_velocity
        self.acceleration         = acceleration
        self.angular_acceleration = angular_acceleration
    
    
    def update(self,dt):
        key = getKey()

        velocity = self.velocity
        if key == FORWARD:
            velocity = self.velocity + self.acceleration * dt
        elif key == BACKWARD:
            velocity = self.velocity - self.acceleration * dt
        else:
            velocity *= .8


        self.velocity = clamp(velocity, -self.max_velocity, self.max_velocity)
        
        angular_velocity = self.angular_velocity
        
        if key == LEFT:
            angular_velocity = self.angular_velocity + self.angular_acceleration * dt
        elif key == RIGHT:
            angular_velocity = self.angular_velocity - self.angular_acceleration * dt
        else:
            angular_velocity *= .7

        self.angular_velocity = clamp(angular_velocity, -self.max_angular_velocity, self.max_angular_velocity)


        cmd_vel = Twist()
        cmd_vel.linear.x = self.velocity
        cmd_vel.angular.z= self.angular_velocity
        
        return  cmd_vel



def main():
    # Add here the name of the ROS node. In ROS, node names must be unique.
    rospy.init_node('robot_teleoperation_node')

    # Publish messages
    pub_vel = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=10)
    # pub_pan = rospy.Publisher('/robot/joint_pan_position_controller/command', Float64, queue_size = 1)
    # pub_tilt = rospy.Publisher('/robot/joint_tilt_position_controller/command', Float64, queue_size = 1)
    
    rate  = rospy.Rate(30)
    robot = Robot()
    
    time  = rospy.Time.now()

    while not rospy.is_shutdown():
        time2= rospy.Time.now()
        dt= time2 - time
        
        cmd_vel = robot.update(dt.to_sec())
        
        time = time2

        pub_vel.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        main()
    except rospy.ROSInternalException:
        pass
    finally:
        print("excecution terminated")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

