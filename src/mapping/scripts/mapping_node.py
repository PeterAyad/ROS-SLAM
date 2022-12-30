#!/usr/bin/python

import rospy
import numpy as np
import message_filters

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan, PointCloud2

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_ros import Buffer, TransformListener
from sensor_msgs import point_cloud2
from laser_geometry import LaserProjection
from skimage.draw import line_aa

pub = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=10)
current_map = np.ones((5000, 5000))*-5

# # Cloud to scan
# def cloud_to_scan(scan):
#     tf = Buffer()
#     tf_listener = TransformListener(tf)

# 0 -> 361 front + 0 -> 361 rear : xls

def callback(scanFront, scanRear, odom):
    # global current_map
    # rospy.loginfo("callback")
    # rospy.loginfo("odom: " + str(odom.pose.pose.position.x) + " " + str(odom.pose.pose.position.y))
    # rospy.loginfo(scanFront)
    # rospy.loginfo(scanRear)
    
    # rospy.loginfo("sending map")
    mapObject = OccupancyGrid()
    mapObject.header.frame_id = "robot_map"
    # mapObject.header.
    mapObject.info.width = 10
    mapObject.info.height = 10
    mapObject.info.resolution = 1
    mapObject.info.origin.position.x = 0
    mapObject.info.origin.position.y = 0
    mapObject.info.origin.position.z = 0
    mapObject.info.origin.orientation.x = 0
    mapObject.info.origin.orientation.y = 0
    mapObject.info.origin.orientation.z = 0
    mapObject.info.origin.orientation.w = 1
    # 0 is white = empty
    # 100 is black = walls
    # -1 is grey = unknown
    mapObject.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 100, 100, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    pub.publish(mapObject)


def main():

    rospy.init_node('mapping_node', anonymous=True)
    scanFront = message_filters.Subscriber('/robot/front_laser/scan', LaserScan)
    scanRear = message_filters.Subscriber('/robot/rear_laser/scan', LaserScan)
    odom = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)

    messageFilter = message_filters.ApproximateTimeSynchronizer([scanFront, scanRear, odom], 10, 0.1, allow_headerless=True)

    messageFilter.registerCallback(callback)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
