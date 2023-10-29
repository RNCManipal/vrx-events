#!/usr/bin/env python3


import math
import numpy
import rclpy
import tf
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from PIDCONTROLLERCLASS import PIDController
from transformations import euler_from_quaternion, quaternion_from_euler
from rcl_interfaces.srv import ListParameters, DescribeParameters, GetParameters, SetParameters


        
        

def normalize_angle(theta):
		
	theta_norm = (theta + math.pi) % (2 * math.pi) - math.pi
	return theta_norm

def gps_to_enu(lat, lon, alt=0):
   		
# 		# Local coordinate origin (Sydney International Regatta Centre, Australia)
# 	lat0 = -33.724223 # degree North
# 	lon0 = 150.679736 # degree East
# 	alt0 = 0 # meters

    # Define a reference point (origin) in your local ENU frame
    reference_lat = -33.724223
    reference_lon = 150.679736

    # Calculate the geodesic distance and azimuth between the reference point and the GPS coordinates
    geod = Geodesic.WGS84.Inverse(reference_lat, reference_lon, lat, lon)

    # Calculate the ENU coordinates
    east = geod['s12']  # East direction
    north = geod['azi1']  # North direction
    up = alt  # Up direction (altitude)

    return north, east, up
	
# 	x = lat0
# 	y = lon0
# 	z = enu[2]
# 	return x, y, z




# last updated oct 16 20:37 all the callbacks working


class Wayfinding(Node):
    def __init__(self):
        super().__init__('thrusters')
        self.cur_pos      = None # Current 2D position (x, y)
        self.cur_rot      = None # Current 1D orientation (yaw)
        self.cur_position = None # Current 3D position (x, y, z)
        self.cur_rotation = None # Current 3D orientation (roll, pitch, yaw)
        self.wps_pos_x    = [] # List of desired position (x) of all waypoints
        self.wps_pos_y    = [] # List of desired position (y) of all waypoints
        self.wps_rot_z    = [] # List of desired orientation (yaw) of all waypoints
        self.wps_position = [] # List of desired 3D position (x, y, z) of all waypoints
        self.wps_rotation = [] # List of desired 3D orientation (roll, pitch, yaw) of all waypoints
        self.wp_count     = None # Waypoint count (starts from 1)
        self.wp_index     = 0 # Waypoint index (starts from 0)
        self.cmd_pos      = None # Commanded 2D position (x, y)
        self.cmd_rot      = None # Commanded 1D orientation (yaw)
        self.time         = None # Current timestamp
        self.config       = {} # Wayfinding configuration
        # ROS infrastructure
        self.tf_broadcaster = None
        self.cmd_vel_msg    = None
        self.cmd_vel_pub    = None
        self.dyn_reconf_srv = None
        
        self.gps_offset = 0.85
        
        self.tf_buffer = Buffer()
        
        
        

        self.imu_Sub = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10
        )
        
        
        self.gps_Sub = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            10
        )

        self.gps_Sub = self.create_subscription(
            PoseArray,
            'vrx/wayfinding/waypoints',
            self.wayfinding_callback,
            10
        )


        self.lateral_thrust_angle_Pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/middle/pos',
            1
        )

        self.lateral_thrust_cmd_Pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/middle/thrust',
            1

        )

        self.left_thrust_angle_Pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/left/pos',
            1
        )

        self.left_thrust_cmd_Pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/left/thrust',
            1
        )

        self.right_thrust_angle_Pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/right/pos',
            1
        )

        self.right_thrust_cmd_Pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/right/thrust',
            1
        )
	
	
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.geometry_msg_point_pub = self.create_publisher(
            PointStamped,
            'geometry_msgs/msg/Point',
            10
        )


		
    def gps_callback(self, msg):
        # if self.yaw is None: # If no yaw data is available, GPS offset cannot be compensated
        #    return
        lat = msg.latitude
        lon = msg.longitude
        pos_x, pos_y, pos_z = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        pos_x += self.gps_offset * math.cos(self.yaw)
        pos_y += self.gps_offset * math.sin(self.yaw)
        self.cur_pos = numpy.array([pos_x, pos_y])
        self.cur_position = numpy.array([pos_x, pos_y, pos_z])
        self.cur_rotation = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        # self.get_logger().info(f" 2D: {self.cur_pos}    3D: {self.cur_position} ROt: {self.cur_rotation}")
        # self.get_logger().info(f" x,y,z {pos_x} {pos_y} {pos_z}")


    def imu_callback(self, msg):

        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        try:
            euler_angles = euler_from_quaternion(quaternion)
            self.roll, self.pitch, self.yaw = euler_angles
            # self.get_logger().info(f"3D Euler Angles: Roll={self.roll}, Pitch={self.pitch}, Yaw={self.yaw}")
        except Exception as e:
            self.get_logger().error(f"Error converting quaternion to Euler angles: {e}")


        
    def goal_callback(self, msg):
#     #     # Transform goal pose from GPS to ENU frame of reference
        lat = msg.pose.position.x
        lon = msg.pose.position.y
        pos_x, pos_y, pos_z = gps_to_enu(lat, lon)

        orientation = msg.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        self.cmd_rot = euler_from_quaternion(quaternion)
        self.cmd_pos = numpy.array([pos_x, pos_y])
        self.cmd_position = numpy.array([pos_x, pos_y, pos_z])
        self.cmd_rotation = quaternion_from_euler(self.cmd_rot[0], self.cmd_rot[1], self.cmd_rot[2])
        err=self.cmd_pos-self.cur_pos
        # total_err = err[0] + 0.75 * err[1]
        self.time = self.get_clock().now().nanoseconds
        # self.get_logger().info(f"cmdpos {self.cmd_pos}  3dcmd {self.cmd_position} ROTcmd {self.cmd_rotation}")
        self.get_logger().info(f"Error at time {self.time}: {err}")

    def wayfinding_callback(self, msg):
        for i in range(len(msg.poses)):
            if msg.poses: # Sanity check
                self.wp_count = len(msg.poses)
                waypoint = msg.poses[i] # Indexing starts from 0
                lat = waypoint.position.x
                lon = waypoint.position.y
                pos_x, pos_y, pos_z = gps_to_enu(lat, lon)

                quaternion = [waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z, waypoint.orientation.w]
                rot_x,rot_y,rot_z = euler_from_quaternion(quaternion)
                
                self.wps_pos_x.append(pos_x)
                self.wps_pos_y.append(pos_y)
                self.wps_rot_z.append(rot_z)
                self.wps_position.append(numpy.array([pos_x, pos_y, pos_z]))
                self.wps_rotation.append(quaternion_from_euler(rot_x, rot_y, rot_z))
                self.get_logger().info(f" x cords !!!!!!!!!!! {self.wps_pos_x}  y corrdinates !!!!!!!!!!! {self.wps_pos_y} z cord !!!!!!!!!!!!!! {self.wps_rot_z}")



    
        

         
def main(args=None):
    rclpy.init(args=args)

    try:
        usv = Wayfinding()
        rclpy.spin(usv)
    except rclpy.exceptions.ROSInterruptException:
        pass

    usv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

