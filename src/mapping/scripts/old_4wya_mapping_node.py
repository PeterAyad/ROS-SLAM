#!/usr/bin/python

import rospy
import numpy as np
import message_filters

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan, PointCloud2

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_ros import Buffer, TransformListener
from sensor_msgs import point_cloud2
from laser_geometry import LaserProjection
from skimage.draw import line_aa
from sensor_msgs import point_cloud2


dimension = 4000
robotPose: Pose = Pose()
current_map = np.ones((dimension, dimension)).astype(np.int8) * -1
laserFront = LaserScan()
laserRear = LaserScan()
publisher = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=10)
# odom = Odometry()


# 0 -> 361 front + 0 -> 361 rear : xls

# def callback(scanFront, scanRear):
#     tf = Buffer()
#     tf_listener = TransformListener(tf)
#     tf_listener.waitForTransform('robot_base_link', 'robot_front_laser_link', rospy.Time(), rospy.Duration(1.0))
#     laserFront = tf_listener.transformLaserScan('robot_base_link', scanFront)
#     # # same for rear
#     tf = Buffer()
#     tf_listener = TransformListener(tf)
#     tf_listener.waitForTransform('robot_base_link', 'robot_rear_laser_link', rospy.Time(), rospy.Duration(1.0))
#     laserRear = tf_listener.transformLaserScan('robot_base_link', scanRear)
# # same for odom
# tf = Buffer()
# tf_listener = TransformListener(tf)
# tf_listener.waitForTransform('robot_base_link', 'robot_odom', rospy.Time(), rospy.Duration(1.0))
# odom = tf_listener.transformOdometry('robot_base_link', odom)
# laserFront = scanFront
# rospy.loginfo(laserFront)
# laserRear = scanRear
# rospy.loginfo(laserRear)


'''
0. pose = /robot/initialpose
1. create matrix center at robot position /robot/initialpose
2. subscribe to /robot/front_laser/scan and /robot/rear_laser/scan
4. tranform laser scan to robot_base_link
3. subscribe to /robot/robotnik_base_control/odom
4. update pose based on odometry
5. update map based on laser scan
    1. ranges 
    2. position
    3. loop on angles from center (position)
        1. get x,y from angle and range
        2. draw line from center to x,y
'''

# my header
# header:
#   seq: 197
#   stamp:
#     secs: 21
#     nsecs: 913000000
#   frame_id: "/robot_map"
# angle_min: -2.359999895095825
# angle_max: 2.359999895095825
# angle_increment: 0.005799999926239252
# time_increment: 0.0
# scan_time: 0.03333333134651184
# range_min: 0.44999998807907104
# range_max: 25.0
# range: [........]


def process_map():
    global current_map, laserFront, laserRear, robotPose, publisher, multiLaser, mergedCloud

    resolution = 1e-2

    laserFront.ranges = np.array(laserFront.ranges)//resolution
    laserRear.ranges = np.array(laserRear.ranges)//resolution

    multiLaser.ranges = np.array(multiLaser.ranges)//resolution

    mapObject = OccupancyGrid()
    mapObject.header.frame_id = "robot_map"
    mapObject.info.width = dimension
    mapObject.info.height = dimension
    mapObject.info.resolution = resolution
    mapObject.info.origin.position.x = -dimension/2 * resolution
    mapObject.info.origin.position.y = -dimension/2 * resolution
    mapObject.info.origin.position.z = 0
    mapObject.info.origin.orientation.x = 0
    mapObject.info.origin.orientation.y = 0
    mapObject.info.origin.orientation.z = 0
    mapObject.info.origin.orientation.w = 1

    # 0 is white = empty
    # 100 is black = walls
    # -1 is grey = unknown

    # print("robotPose", robotPose)
    try:

        # draw points in mergedCloud
        for p in point_cloud2.read_points(mergedCloud, skip_nans=True):
            print("p", p)
            current_map[int((robotPose.position.x -p[0])//resolution)
                        ][int((robotPose.position.x - p[1])//resolution)] = 0

        # shiftTheta = robotPose.orientation.z
        # startTheta = multiLaser.angle_min - shiftTheta
        # endTheta = multiLaser.angle_max - shiftTheta
        # step = multiLaser.angle_increment
        # end = len(laserFront.ranges)
        # theta = startTheta
        # index = 0
        # while theta < endTheta and index < end:
        #     x = robotPose.position.x - \
        #         laserFront.ranges[index] * np.cos(theta) + dimension/2
        #     # x*=100
        #     y = robotPose.position.y - \
        #         laserFront.ranges[index] * np.sin(theta) + dimension/2
        #     # y*=100
        #     print(x, y)
        #     # rr, cc, val = line_aa(int(robotPose.position.x),
        #     #                       int(robotPose.position.y), int(x), int(y))
        #     # current_map[rr, cc] = 100
        #     current_map[int(x), int(y)] = 100
        #     theta += step
        #     index += 1

        # theta = -2.359999895095825
        # index = 0
        # while theta < 2.359999895095825 and index < 361:
        #     x = robotPose.position.x - \
        #         laserFront.ranges[index] * np.cos(theta) + dimension/2
        #     # x*=100
        #     y = robotPose.position.y - \
        #         laserFront.ranges[index] * np.sin(theta) + dimension/2
        #     # y*=100
        #     print(x, y)
        #     # rr, cc, val = line_aa(int(robotPose.position.x),
        #     #                       int(robotPose.position.y), int(x), int(y))
        #     # current_map[rr, cc] = 100
        #     current_map[int(x), int(y)] = 100
        #     theta += 0.005799999926239252
        #     index += 1

        # theta = -2.359999895095825
        # index = 0
        # while theta < 2.359999895095825 and index < 361:
        #     x = robotPose.position.x - \
        #         laserRear.ranges[index] * np.cos(theta) + dimension/2
        #     # x*=100
        #     y = robotPose.position.y - \
        #         laserRear.ranges[index] * np.sin(theta) + dimension/2
        #     # y*=100
        #     # print(x,y)
        #     # rr, cc, val = line_aa(int(robotPose.position.x),
        #     #                       int(robotPose.position.y), int(x), int(y))
        #     # current_map[rr, cc] = 100
        #     current_map[int(x), int(y)] = 100
        #     theta += 0.005799999926239252
        #     index += 1

        # for i in range(1000,1100):
        #     for j in range(1000,2000):
        #         current_map[i,j] = 100

        current_map = np.array(current_map).astype(np.int8)
        mapObject.data = current_map.flatten().tolist()

        publisher.publish(mapObject)
        # rospy.loginfo(mapObject)
    except Exception as e:
        print(e)


def poseCallback(modelState):
    global robotPose
    robotPose = modelState.pose[2]


def rearLaserCallback(laser):
    global laserRear
    # tf = Buffer()
    # tf_listener = TransformListener(tf)

    # tf_listener.waitForTransform('robot_base_link', 'robot_rear_laser_link', rospy.Time(), rospy.Duration(1.0))
    # laserRear = tf_listener.transformLaserScan('robot_base_link', laser)
    laserRear = laser


def frontLaserCallback(laser):
    global laserFront
    # tf = Buffer()
    # tf_listener = TransformListener(tf)
    # trans = tf_listener.lookupTransform('robot_base_link', 'robot_front_laser_link', rospy.Time())
    # # tf_listener.waitForTransform('robot_base_link', 'robot_front_laser_link', rospy.Time(), rospy.Duration(1.0))
    # laserFront = tf_listener.transformLaserScan('robot_base_link', laser)
    laserFront = laser


multiLaser = LaserScan()


def multiLaserCallback(laser):
    global multiLaser
    multiLaser = laser


mergedCloud = PointCloud2()


def mergedCloudCallback(cloud):
    global mergedCloud
    mergedCloud = cloud


def main():
    rospy.init_node('mapping_node', anonymous=True)
    # publisher = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=10)

    rospy.Subscriber('/gazebo/model_states', ModelStates, poseCallback)

    rospy.Subscriber('/robot/front_laser/scan', LaserScan, frontLaserCallback)
    rospy.Subscriber('/robot/rear_laser/scan', LaserScan, rearLaserCallback)

    rospy.Subscriber('/scan_multi', LaserScan, multiLaserCallback)

    rospy.Subscriber('/merged_cloud', PointCloud2, mergedCloudCallback)

    # odom = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)

    # messageFilter = message_filters.ApproximateTimeSynchronizer(
    #     [scanFront, scanRear], 10, 0.1, allow_headerless=True)
    # messageFilter.registerCallback(callback)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        process_map()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
