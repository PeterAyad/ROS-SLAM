#!/usr/bin/env python3

import numpy as np
import message_filters
import rospy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from sensor_msgs import point_cloud2
from laser_geometry import LaserProjection
from mapping.utils import log_2_probability, probability_2_log, is_inside_mat

from skimage.draw import line


def laser_world_points(source_laser_frame):
    cloud = LaserProjection().projectLaser(source_laser_frame)
    world_points = list(point_cloud2.read_points(
        cloud, field_names=("x", "y"), skip_nans=True))
    return world_points


class Map:
    frame_id = 'robot_map'

    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution

        self.world = sensor_model_l_prior * \
            np.ones((self.width_in_pxs, self.height_in_pxs), dtype=np.float32)

        self.origin_x = -self.width//2
        self.origin_y = -self.height//2

    @property
    def width_in_pxs(self):
        return int(self.width/self.resolution)

    @property
    def height_in_pxs(self):
        return int(self.height/self.resolution)

    def _to_map_pixels(self, x, y):
        return int((x-self.origin_x)/self.resolution), int((y-self.origin_y)/self.resolution)

    def draw_line(self, x0, y0, x1, y1):
        """
            updates the propabilities of the points in line 
            starting from x0,y0 to x1,y1 in the world matrix
        """
        x0, y0 = self._to_map_pixels(x0, y0)
        x1, y1 = self._to_map_pixels(x1, y1)

        rr, cc = line(x0, y0, x1, y1)

        # check if points inside the map

        valid_points = (rr < self.world.shape[0]) & (
            rr >= 0) & (cc < self.world.shape[1]) & (cc >= 0)

        rr = rr[valid_points]
        cc = cc[valid_points]

        # free points
        self.world[rr, cc] += sensor_model_l_free - sensor_model_l_prior

        if len(rr) == 0:
            return 0, 0

        # return the last point of the line
        return rr[-1], cc[-1]

    def as_occ_grid(self, stamp, frame_id='robot_map'):
        map_occ = OccupancyGrid()
        map_occ.header.frame_id = frame_id
        map_occ.info.resolution = self.resolution

        map_occ.info.width = self.width_in_pxs
        map_occ.info.height = self.height_in_pxs
        map_occ.info.origin.position.x = self.origin_x
        map_occ.info.origin.position.y = self.origin_y
        map_occ.header.stamp = stamp

        gridmap = self.world.copy()
        gridmap = gridmap.T
        gridmap_p = log_2_probability(gridmap)
        gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
        map_occ.data = gridmap_int8.flatten()
        return map_occ


x = 0
y = 0
theta = 0
time = 0


def get_robot_position(odom):
    '''
    input : odom (vx_odom, vy_odom, w_odom)
            x_sensor, y_sensor, theta_sensor
    output : x, y, theta
    '''

    global x, y, theta, time

    if time == 0:
        time = odom.header.stamp
        return 0, 0

    dt = (odom.header.stamp - time).to_sec()
    time = odom.header.stamp

    print("dt: ", dt)

    # kallman gain
    k = 0.79

    # velocities from odom frame to sensor frame
    vx = odom.twist.twist.linear.x * \
        np.cos(odom.twist.twist.angular.z) - \
        odom.twist.twist.linear.y * np.sin(odom.twist.twist.angular.z)
    if vx < 0.0001:
        vx = 0
    vy = odom.twist.twist.linear.x * \
        np.sin(odom.twist.twist.angular.z) + \
        odom.twist.twist.linear.y * np.cos(odom.twist.twist.angular.z)
    if vy < 0.0001:
        vy = 0
    w = odom.twist.twist.angular.z
    if w < 0.0001:
        w = 0

    print("vx: ", vx, "vy: ", vy, "w: ", w)

    # Predicted robot position
    #
    xp = x + vx * dt
    yp = y + vy * dt
    thetap = theta + w * dt

    print("xp: ", xp, "yp: ", yp, "thetap: ", thetap)

    # Corrected robot position
    #
    # read sensor position
    x_sensor = odom.pose.pose.position.x
    y_sensor = odom.pose.pose.position.y
    theta_sensor = odom.pose.pose.orientation.z

    print("x_sensor:", x_sensor)
    print("y_sensor:", y_sensor)
    print("theta_sensor", theta_sensor)

    #
    # correct using kallman gain
    x = xp + k * (x_sensor - xp)
    y = yp + k * (y_sensor - yp)
    theta = thetap + k * (theta_sensor - thetap)

    print("x: ", x, "y: ", y, "theta: ", theta)
    return x, y


def mapping_callback(scan,  odom):
    rospy.loginfo("Mapping Callback Handler Called")

    global resolution

    world_points = laser_world_points(scan)
    world_points = np.array(world_points).astype(dtype=np.float32)

    # put the world points in the world array
    # loop over the world_points
    for i, point in enumerate(world_points):

        d = scan.ranges[i] / resolution

        robot_x, robot_y = get_robot_position(odom)

        y, x = map.draw_line(robot_x, robot_y, point[0], point[1])

        if not np.isnan(d) and d != SENSOR_MAX_RANGE and is_inside_mat(y, x, map.world):
            map.world[y, x] += sensor_model_l_occ - sensor_model_l_prior

    occ_grid_publisher.publish(map.as_occ_grid(stamp=scan.header.stamp))
    rospy.loginfo("[MappingModule] Published the map")


resolution = 0.25  # for fast mapping
# resolution = 0.02 # for high resolution mapping

SENSOR_MAX_RANGE = 30

if __name__ == '__main__':

    sensor_model_l_occ = probability_2_log(0.75)
    sensor_model_l_prior = probability_2_log(0.5)
    sensor_model_l_free = probability_2_log(0.45)

    rospy.init_node('MappingModule', anonymous=True)

    # time = rospy.Time.now()

    map = Map(100, 100, resolution)

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    rospy.loginfo("[MappingModule] node started")

    # subscriber to the laser scan topic
    laser_sub = message_filters.Subscriber('/scan_multi', LaserScan)

    # publisher to the occupancy grid topic
    occ_grid_publisher = rospy.Publisher(
        '/robot/map', OccupancyGrid, queue_size=10)

    # subscriber to the odometry topic
    odom_sub = message_filters.Subscriber(
        '/robot/robotnik_base_control/odom', Odometry)

    # Create a time synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, odom_sub],
                                                     10, 0.1, allow_headerless=True)

    ts.registerCallback(mapping_callback)

    while not rospy.is_shutdown():
        rospy.spin()
