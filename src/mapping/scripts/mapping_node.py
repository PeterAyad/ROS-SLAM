#!/usr/bin/python

import rospy
from nav_msgs.msg import OccupancyGrid


def main():
    pub = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=10)
    rospy.init_node('mapping_node', anonymous=True)
    rate = rospy.Rate(1)
    rospy.loginfo("I am a mapping node")
    while not rospy.is_shutdown():
        rospy.loginfo("sending map")
        mapObject = OccupancyGrid()
        mapObject.header.frame_id = "map"
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
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass