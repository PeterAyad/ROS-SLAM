# #! /usr/bin/env python

# import rospy
# import numpy as np
# import tf
# import math

# import message_filters
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import Pose
# from nav_msgs.msg import OccupancyGrid
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion

# def get_line(start, end):
#     """Bresenham's Line Algorithm
#     Produces a list of tuples from start and end
 
#     >>> points1 = get_line((0, 0), (3, 4))
#     >>> points2 = get_line((3, 4), (0, 0))
#     >>> assert(set(points1) == set(points2))
#     >>> print points1
#     [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
#     >>> print points2
#     [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
#     """
#     # Setup initial conditions
#     x1, y1 = start
#     x2, y2 = end
#     dx = x2 - x1
#     dy = y2 - y1
 
#     # Determine how steep the line is
#     is_steep = abs(dy) > abs(dx)
 
#     # Rotate line
#     if is_steep:
#         x1, y1 = y1, x1
#         x2, y2 = y2, x2
 
#     # Swap start and end points if necessary and store swap state
#     swapped = False
#     if x1 > x2:
#         x1, x2 = x2, x1
#         y1, y2 = y2, y1
#         swapped = True
 
#     # Recalculate differentials
#     dx = x2 - x1
#     dy = y2 - y1
 
#     # Calculate error
#     error = int(dx / 2.0)
#     ystep = 1 if y1 < y2 else -1
 
#     # Iterate over bounding box generating points between start and end
#     y = y1
#     points = []
#     for x in range(x1, x2 + 1):
#         coord = (y, x) if is_steep else (x, y)
#         points.append(coord)
#         error -= abs(dy)
#         if error < 0:
#             y += ystep
#             error += dx
 
#     # Reverse the list if the coordinates were swapped
#     if swapped:
#         points.reverse()
#     return points


# # --------------- INITIALIZATIONS ----------------------

# n_height = 5000  # n de celulas
# n_width = 5000  # n de celulas
# resolution = 1  # resolucao em metros
# min_angle = -3/4*np.pi  # em radianos
# max_angle = 3/4*np.pi  # em radianos
# n_beams = 794

# # Mapa com as probabilidades
# ogm_map = np.zeros((n_height, n_width))
# lidar_angles = np.linspace(min_angle, max_angle, n_beams)

# lti_matrix = np.zeros((n_height, n_width))

# x_origin = -15
# y_origin = -17

# a = np.linspace(x_origin, y_origin+n_width*resolution, num=n_width)
# xi = np.vstack([a] * n_width)
# yi = np.hstack([np.transpose(a[np.newaxis])] * n_height)


# # --------------------- FUNCTIONS ----------------------

# # 3D Transf
# # roll  -> x axis   ->alpha
# # pitch -> y axis   ->beta
# # yaw   -> z axis   ->gamma

# # Receives (x,y,z) and returns (x',y',z')=[R](x,y,z)
# # given pitch, yaw and roll
# # in radians, because of numpy
# # point is a

# def Euler_rotation(roll, pitch, yaw, point):
#     R_alpha = np.matrix([[1, 0,              0],
#                         [0, np.cos(roll),   -np.sin(roll)],
#                         [0, np.sin(roll),   np.cos(roll)]])
#     R_beta = np.matrix([[np.cos(pitch), 0, np.sin(pitch)],
#                         [0, 1, 0],
#                         [-np.sin(pitch), 0, np.cos(pitch)]])

#     R_gamma = np.matrix([[np.cos(yaw), -np.sin(yaw), 0],
#                         [np.sin(yaw), np.cos(yaw), 0],
#                         [0, 0, 1]])

#     # This order of matrix mult. was tested in geogebra
#     R = np.matmul(R_alpha, R_beta, R_gamma)

#     return R.dot(point)

# # Returns (x,y) of 2D polar data


# def polar_to_rect(range, teta):
#     return ([range*np.cos(teta), range*np.sin(teta)])

# # To obtain the norm of 2D or 3D vector's, use this:
# # numpy.linalg.norm


# class Mapping(object):

#     def __init__(self):


#         self.drone_pose = Pose()
#         self.lidar_points = []

#     def callback(self, lidar_msg, pose_msg):
#         # rospy.loginfo("Size of lidar_msg: %d", len(lidar_msg.ranges))
#         self.lidar_points = lidar_msg.ranges
#         self.drone_pose = pose_msg.pose.pose

#         # quaternion = self.drone_pose.orientation
#         # explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
#         # roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
#         # print("roll: ", roll, "pitch: ", pitch, "yaw: ", yaw)

#     # def lidar_callback(self,msg):
#     #     self.lidar_points = msg.ranges

#     # def pose_callback(self, msg):
#     #     self.drone_pose = msg.pose

#     # The Inverse Range Sensor Model will return a matrix with Logaritmic Values and we have to put that values as Proabilities
#     # p(m|z,x) = 1-(1/(1+exp{log_odds}))

#     def log_to_prob(self, matrix):
#         exp_logodds = np.exp(matrix)
#         probability = 1 - np.divide(1, 1+exp_logodds)
#         return probability

#     def inverse_range_sensor_model(self, x, y, yaw, zt):

#         np.set_printoptions(threshold=np.inf)

#         occupancy = np.zeros((n_width, n_height))

#         print("x:", x, " y:", y, " yaw:", yaw)

#         zmax = 10.0
#         alpha = 0.02
#         beta = 0.009817477*2  # 2pi/640 - distance between adjacent beams

#         # x_mat = np.full((n_width,n_height),x) # array of x values
#         # y_mat = np.full((n_width,n_height),y) # array of y values

#         # r = np.sqrt(np.square(xi - x_mat) + np.square(yi - y_mat)) # relative range
#         # print("r :")
#         # print(r)
#         # phi = np.arctan2(yi - y_mat,xi - x_mat) - yaw # relative heading
#         # print("phi :")
#         # print(phi)

#         # Iterating through every cell, checking their state of occupancy
#         # for i in range(0,n_height):
#         # for j in range(0,n_width):
#         # Findes the index correponding to the laser beam that intersects the cell
#         # k = np.argmin(np.absolute(lidar_angles - phi[i][j]), axis=-1)

#         x += -x_origin  # in meters
#         y += -y_origin  # in meters
#         x_pos = int(x/resolution)  # in pixels
#         y_pos = int(y/resolution)  # in pixels
#         for i in range(n_beams):

#             # Corresponding range
#             z = zt[i]
#             # For now we will ignore the unknown positions

#             # but is possible t
#             if math.isnan(z):  # if there was a reading error
#                 continue

#             limit = True  # if we reveive a inf, we know that the space between the zmax
#             # and the drone is empty. So we can map it as empty
#             if z == float('inf'):
#                 z = zmax - (alpha)
#                 limit = False

#             # (target_x,target_y) is the position that the laser is reading, if the
#             # drone is at (0,0). Further translation is necessary
#             target = polar_to_rect(z, lidar_angles[i]+yaw)

#             # points is a list (converted to tuple for more ) of all the points that compose the beam (like a pixelized line in paint)
#             points = get_line((x_pos, y_pos), (int(
#                 (target[0]+x)/resolution), int((target[1]+y)/resolution)))

#             # print(points)

#             for j in range(len(points)):

#                 # check if the calculation is within map cell bounds

#                 if ((points[j][0] >= n_height) or (points[j][1] >= n_width) or (points[j][0] < 0) or (points[j][1] < 0)):
#                     continue

#                 # print("x =",points[j][0],"  y =",)

#                 z_new = resolution*math.sqrt(((points[j][0]-x_pos)*(points[j][0]-x_pos))+(
#                     (points[j][1]-y_pos)*(points[j][1]-y_pos)))
#                 if limit and z < zmax and abs(z_new - z) < alpha/2:
#                     occupancy[points[j][0]][points[j][1]] = 1  # Occupied
#                 elif z_new <= z:
#                     occupancy[points[j][0]][points[j][1]] = -1  # Free

#         return occupancy

#     # Lets define a mapping with Occupancy Grid Mapping

#     def map_with_OGM(self, lidar_points, drone_pose):
#         # rospy.loginfo("Mapping with OGM")

#         global n_width, n_height, resolution, lti_matrix

#         # First lets peparare auxiliar variables
#         # 1) rotation about Zaxis more known as yaw
#         quaternion = drone_pose.orientation
#         explicit_quat = [quaternion.x, quaternion.y,
#                          quaternion.z, quaternion.w]
#         roll, pitch, yaw = tf.transformations.euler_from_quaternion(
#             explicit_quat)
#         euler = (roll, pitch, yaw)

#         # 2) Using a python and ROS nav_msgs::msg::_OccupancyGrid
#         OGM = OccupancyGrid()
#         # Set up this function about the header
#         OGM.header.stamp = rospy.Time.now()
#         OGM.header.frame_id = "robot_base_link"
#         OGM.header.seq = 0
#         # Set up this function about the info
#         OGM.info.resolution = resolution
#         OGM.info.width = n_width
#         OGM.info.height = n_height
#         OGM.info.map_load_time = rospy.Time.now()
#         # Set up the map origin
#         OGM.info.origin.position.x = x_origin
#         OGM.info.origin.position.y = y_origin

#         # Lets now write the OGM algorithm to use in the
#         if len(lidar_points) != 0 and abs(roll) < 0.05 and abs(pitch) < 0.05:

#             # If the vector its not empty we should update the matrix with the help of the inverse range sensor model
#             # l(t,i) = l(t-1,i) + inverse range-sensor model - l0 (l0 = 0)
#             # Note: the l matriz was defined in the inverse ange sensor code with enteries like  positionx, positiony, theta, lidar_points
#             lti_matrix = np.add(lti_matrix, self.inverse_range_sensor_model(
#                 drone_pose.position.x, drone_pose.position.y, yaw, lidar_points))
#             # Converter de logaritmo para probabilidade
#             Prob_Matrix = self.log_to_prob(lti_matrix)
#             # Colocar em 1 dimensao a matriz
#             Prob_Matrix_1D = np.concatenate(Prob_Matrix, axis=0)
#             # Guardar as probabilidades no OGM definido em cima
#             for i in range(len(Prob_Matrix_1D)):
#                 # Multiply by 100 to obtain a 0-100% probabilities
#                 OGM.data.append(Prob_Matrix_1D[i] * 100)

#         # Publish a topic with the map
#         OGM_publisher = rospy.Publisher(
#             '/robot/map', OccupancyGrid, queue_size=1)
#         OGM_publisher.publish(OGM)


# # ---------------------- MAIN ------------------------------------

# def main():
#     # declare the node name
#     rospy.init_node('mapping', anonymous=True)

#     mapping = Mapping()

#     # lidarSub = rospy.Subscriber('/iris_0/scan', LaserScan,mapping.lidar_callback, queue_size = 1)
#     # poseSub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, mapping.pose_callback, queue_size = 1)

#     lidarSub = message_filters.Subscriber('/scan_multi', LaserScan)
#     poseSub = message_filters.Subscriber(
#         '/robot/robotnik_base_control/odom', Odometry)

#     ts = message_filters.ApproximateTimeSynchronizer(
#         [lidarSub, poseSub], queue_size=1, slop=0.05)
#     ts.registerCallback(mapping.callback)

#     rate = rospy.Rate(0.02)
#     while not rospy.is_shutdown():
#         # rospy.loginfo("Mapping node started")
#         mapping.map_with_OGM(mapping.lidar_points, mapping.drone_pose)
#         rate.sleep()


# if __name__ == '__main__':
#     main()


# # header:
# #  seq: 380
# #  stamp:
# #    secs: 689
# #    nsecs: 494000000
# #  frame_id: "iris_0/laser_2d"
# # angle_min: -3.1400001049
# # angle_max: 3.1400001049
# # angle_increment: 0.00982785597444
# # time_increment: 0.0
# # scan_time: 0.0
# # range_min: 0.0799999982119
# # range_max: 10.0


# # my header
# # header:
# #   seq: 197
# #   stamp:
# #     secs: 21
# #     nsecs: 913000000
# #   frame_id: "/robot_map"
# # angle_min: -2.359999895095825
# # angle_max: 2.359999895095825
# # angle_increment: 0.005799999926239252
# # time_increment: 0.0
# # scan_time: 0.03333333134651184
# # range_min: 0.44999998807907104
# # range_max: 25.0
# # range: [........]

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


def callback_handler(front_scan, rear_scan, odom):
    """callback function handler for the time synchronizer"""
    rospy.loginfo("Callback handler called")

    world_points = get_transform_cloud(front_scan, odom)
    # extract the data from the messages
    front_laser_reading = np.array(front_scan.ranges[0:361])
    front_min_angle = -90*np.pi/180 + 3*45*np.pi/180
    front_max_angle = 90*np.pi/180 + 3*45*np.pi/180
    front_angle_increment = front_scan.angle_increment
    front_max_range = front_scan.range_max
    front_min_range = front_scan.range_min
    # publish the data, all angles are in radians
    # msg = Data()
    # msg.stamp = front_scan.header.stamp
    # msg.fl_reading = front_laser_reading.astype(dtype=np.float32)
    # msg.fl_min_angle = front_min_angle
    # msg.fl_max_angle = front_max_angle
    # msg.fl_angle_inc = front_angle_increment
    # msg.fl_max_range = front_max_range
    # msg.fl_min_range = front_min_range
    #data_pub.publish(msg)
    # put the world points in the world array
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = 'robot_map'
    map_msg.info.resolution = 0.02
    map_msg.info.width = int(100/0.02)
    map_msg.info.height = int(100/0.02)
    map_msg.info.origin.position.x = -50
    map_msg.info.origin.position.y = -50
    map_msg.header.stamp = front_scan.header.stamp
    world_points = np.array(world_points).astype(dtype=np.float32)
    rospy.loginfo(len(world_points))
    # put the world points in the world array
    # loop over the world_points
    counthits = 0
    for i, point in enumerate(world_points):
        d = front_scan.ranges[i] / 0.02
        # rospy.loginfo(int(point[0]))
        # rospy.loginfo(int(point[1]))
        ip, jp, is_hit = bresenham(int((odom.pose.pose.position.x+50)/0.02), int((odom.pose.pose.position.y+50)/0.02), int((point[0]+50)/0.02), int((point[1]+50)/0.02), d)
        if not np.isnan(d) and d != 30 and is_inside(int(ip),int(jp)):
            # rospy.loginfo(ip)
            # rospy.loginfo(jp)
            counthits += 1
            world[int(ip),int(jp)] += sensor_model_l_occ - sensor_model_l_prior
    rospy.loginfo(counthits)
    gridmap = world.copy()
    gridmap = gridmap.T
    gridmap_p = l2p(gridmap)
    gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
    map_msg.data = gridmap_int8.flatten()
    grid_pub.publish(map_msg)
    rospy.loginfo("Published the map")

def test_cb(front_scan, rear_scan):

    rospy.loginfo("Callback handler called")

    rospy.loginfo("front_scan")
    rospy.loginfo(front_scan)

    rospy.loginfo("rear_scan")
    rospy.loginfo(rear_scan)
    exit(0)
    
def test_cb2(merged_cloud, scan_multi,odom):

    rospy.loginfo("Callback handler called")

    world_points =list(point_cloud2.read_points(merged_cloud, field_names=("x", "y", "z"), skip_nans=True))
    rospy.loginfo("world_points")
    # rospy.loginfo(world_points)

    rospy.loginfo("scan_multi")
    # rospy.loginfo(scan_multi)
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = 'robot_map'
    map_msg.info.resolution = 0.02
    map_msg.info.width = int(100/0.02)
    map_msg.info.height = int(100/0.02)
    map_msg.info.origin.position.x = -50
    map_msg.info.origin.position.y = -50
    map_msg.header.stamp = merged_cloud.header.stamp
    world_points = np.array(world_points).astype(dtype=np.float32)
    rospy.loginfo(len(world_points))
    # put the world points in the world array
    # loop over the world_points
    counthits = 0
    for i, point in enumerate(world_points):
        d = scan_multi.ranges[i] / 0.02
        # rospy.loginfo(int(point[0]))
        # rospy.loginfo(int(point[1]))
        ip, jp, is_hit = bresenham(int((odom.pose.pose.position.x+50)/0.02), int((odom.pose.pose.position.y+50)/0.02), int((point[0]+50)/0.02), int((point[1]+50)/0.02), d)
        if not np.isnan(d) and d != 30 and is_inside(int(ip),int(jp)):
            # rospy.loginfo(ip)
            # rospy.loginfo(jp)
            counthits += 1
            world[int(ip),int(jp)] += sensor_model_l_occ - sensor_model_l_prior
    rospy.loginfo(counthits)
    gridmap = world.copy()
    gridmap = gridmap.T
    gridmap_p = l2p(gridmap)
    gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
    map_msg.data = gridmap_int8.flatten()
    grid_pub.publish(map_msg)
    rospy.loginfo("Published the map")

if __name__ == '__main__':
    sensor_model_l_occ = p2l(0.75)
    sensor_model_l_prior = p2l(0.02)
    sensor_model_l_free = p2l(0.45)
    
    world = sensor_model_l_prior * np.ones((5000, 5000), dtype=np.float32)
    rospy.init_node('mapping', anonymous=True)
    # tf_buffer = Buffer()
    # tf_listener = TransformListener(tf_buffer)
    rospy.loginfo("\n\n\n\nMappingNode started\n\n\n\n")
    # Create a subscriber to the front laser scan topic
    front_laser_sub = message_filters.Subscriber('/robot/front_laser/scan', LaserScan)

    # Create a subscriber to the rear laser scan topic
    rear_laser_sub = message_filters.Subscriber('/robot/rear_laser/scan', LaserScan)
    merged_cloud = message_filters.Subscriber('/merged_cloud', PointCloud2)
    scan_multi = message_filters.Subscriber('/scan_multi', LaserScan)

    # Create a publisher to the occupancy grid topic
    grid_pub = rospy.Publisher('/ourMap', OccupancyGrid, queue_size=10)

    # Create a subscriber to the odometry topic
    odom_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)

    # Create a publisher to the data topic
    # data_pub = rospy.Publisher('/ourTopic', Data, queue_size=10)

    # Create a time synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([merged_cloud, scan_multi,odom_sub],
                                                    10, 0.1, allow_headerless=True)

    # Register a callback with the time synchronizer
    # ts.registerCallback(callback_handler)
    ts.registerCallback(test_cb2)
    while not rospy.is_shutdown():
        rospy.spin()