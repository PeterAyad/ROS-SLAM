#!/usr/bin/env python3

import numpy as np
import message_filters
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_ros import Buffer, TransformListener
from sensor_msgs import point_cloud2
from laser_geometry import LaserProjection
# from cmp_robot.msg import Data


def l2p(l):
    """Convert log-odds to probability."""
    return 1 - (1/(1+np.exp(l)))

# l(x) = log(\frac{p(x)}{1 - p(x)})
def p2l(p):
    """Convert probability to log-odds."""
    return np.log(p/(1-p))


def is_inside (i, j):
    """Check if grid coordinates are inside the map."""
    return i < world.shape[0] and j < world.shape[1] and i>=0 and j>=0

def bresenham (i0, j0, i1, j1, d):   # i0, j0 (starting point)
    """Bresenham's line algorithm."""
    dx = np.absolute(j1-j0)
    dy = -1 * np.absolute(i1-i0)
    sx = -1
    if j0<j1:
        sx = 1
    sy = -1
    if i0<i1:
        sy = 1
    jp, ip = j0, i0
    err = dx+dy                     # error value e_xy
    while True:                     # loop
        if (jp == j1 and ip == i1) or (np.sqrt((jp-j0)**2+(ip-i0)**2) >= d) or not is_inside(ip, jp):
            return ip, jp, False
        elif world[int(ip),int(jp)]==100:
            return ip, jp, True

        if is_inside(ip, jp):
            # miss:
            world[int(ip),int(jp)] += sensor_model_l_free - sensor_model_l_prior

        e2 = 2*err
        if e2 >= dy:                # e_xy+e_x > 0 
            err += dy
            jp += sx
        if e2 <= dx:                # e_xy+e_y < 0
            err += dx
            ip += sy



def get_transform_cloud(source_frame, target_frame):
    """Get the transform from the robot frame to the laser frame."""
    # get the transform from the robot frame to the laser frame
    cloud = LaserProjection().projectLaser(source_frame)
    transform = tf_buffer.lookup_transform(target_frame.header.frame_id, source_frame.header.frame_id, rospy.Time())
    pcd2_laser = do_transform_cloud(cloud, transform) #returns PointCloud2
    world_points = list(point_cloud2.read_points(pcd2_laser, field_names=("x", "y", "z"), skip_nans=True))
    # rospy.loginfo(world_points)
    return world_points

def quarternion_to_yaw(q_x, q_y, q_z, q_w):
    """Convert quarternion to yaw angle."""
    siny_cosp = 2 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
    return np.arctan2(siny_cosp, cosy_cosp)


def callback_handler(front_scan,  odom):
    """callback function handler for the time synchronizer"""
    rospy.loginfo("Callback handler called")

    world_points = get_transform_cloud(front_scan, odom)
    # extract the data from the messages
  
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = 'robot_map'
    map_msg.info.resolution = 0.02
    map_msg.info.width = int(50/0.02)
    map_msg.info.height = int(50/0.02)
    map_msg.info.origin.position.x = -25
    map_msg.info.origin.position.y = -25
    map_msg.header.stamp = front_scan.header.stamp
    world_points = np.array(world_points).astype(dtype=np.float32)
    # put the world points in the world array
    # loop over the world_points
    for i, point in enumerate(world_points):
        d = front_scan.ranges[i] / 0.02
        # rospy.loginfo(int(point[0]))
        # rospy.loginfo(int(point[1]))
        ip, jp, is_hit = bresenham(int((odom.pose.pose.position.x+25)/0.02), int((odom.pose.pose.position.y+25)/0.02), int((point[0]+25)/0.02), int((point[1]+25)/0.02), d)
        if not np.isnan(d) and d != 30 and is_inside(int(ip),int(jp)):
            # rospy.loginfo(ip)
            # rospy.loginfo(jp)
            world[int(ip),int(jp)] += sensor_model_l_occ - sensor_model_l_prior
    gridmap = world.copy()
    gridmap = gridmap.T
    gridmap_p = l2p(gridmap)
    gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
    map_msg.data = gridmap_int8.flatten()
    grid_pub.publish(map_msg)
    rospy.loginfo("Published the map")
    

if __name__ == '__main__':
    sensor_model_l_occ = p2l(0.75)
    sensor_model_l_prior = p2l(0.5)
    sensor_model_l_free = p2l(0.45)
    world = sensor_model_l_prior * np.ones((2500, 2500), dtype=np.float32)
    rospy.init_node('SensorModule', anonymous=True)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    rospy.loginfo("SensorModule node started")
    # Create a subscriber to the front laser scan topic
    front_laser_sub = message_filters.Subscriber('/scan_multi', LaserScan)

    # Create a subscriber to the rear laser scan topic
    # rear_laser_sub = message_filters.Subscriber('/robot/rear_laser/scan', LaserScan)

    # Create a publisher to the occupancy grid topic
    grid_pub = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=10)

    # Create a subscriber to the odometry topic
    odom_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)

    # Create a publisher to the data topic
    # data_pub = rospy.Publisher('/ourTopic', Data, queue_size=10)

    # Create a time synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([front_laser_sub, odom_sub],
                                                    10, 0.1, allow_headerless=True)

    # Register a callback with the time synchronizer
    ts.registerCallback(callback_handler)
    while not rospy.is_shutdown():
        rospy.spin()