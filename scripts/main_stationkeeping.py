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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from PIDCONTROLLERCLASS import PIDController
from transformations import  euler_from_quaternion, quaternion_from_euler
from rcl_interfaces.srv import ListParameters, DescribeParameters, GetParameters, SetParameters



        
        

def normalize_angle(self,theta):
		
	theta_norm = (theta + math.pi) % (2 * math.pi) - math.pi
	return theta_norm

def gps_to_enu(self,lat, lon, alt=0):
   		
		# Local coordinate origin (Sydney International Regatta Centre, Australia)
	lat0 = -33.724223 # degree North
	lon0 = 150.679736 # degree East
	alt0 = 0 # meters
	enu = self.geod.Forward(lat, lon, alt, lat0, lon0, alt0)
	x = enu[0]
	y = enu[1]
	z = enu[2]
	return x, y, z







class StationKeeping(Node):
    def __init__(self):
        super().__init__('thrusters')
        self.cur_pos      = None # Current 2D position (x, y)
        self.cur_rot      = None # Current 1D orientation (yaw)
        self.cur_position = None # Current 3D position (x, y, z)
        self.cur_rotation = None # Current 3D orientation (roll, pitch, yaw)
        self.cmd_pos      = None # Commanded 2D position (x, y)
        self.cmd_rot      = None # Commanded 1D orientation (yaw)
        self.cmd_position = None # Commanded 3D position (x, y, z)
        self.cmd_rotation = None # Commanded 3D orientation (roll, pitch, yaw)
        self.time         = None # Current timestamp
        self.config       = {} # Station-keeping configuration
        # ROS infrastructure
        self.tf_broadcaster = None
        self.cmd_vel_msg    = None
        self.cmd_vel_pub    = None
        self.dyn_reconf_srv = None
        
        
        self.gps_offset = None
        
        self.tf_buffer = Buffer()
        
        
        '''self.Pose_error_Sub = self.create_subscription(
            Float32,
            '/vrx/stationkeeping/pose_error',
            self.pecallback,
            10
        )

        self.Mean_pose_error_Sub = self.create_subscription(
            Float32,
            '/vrx/stationkeeping/mean_pose_error',
            self.mpecallback,
            10
        )'''
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

        


        self.Goal_Sub = self.create_subscription(
            PoseStamped,
            '/vrx/stationkeeping/goal',
            self.goal_callback,
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
        if self.cur_rot is None: # If no yaw data is available, GPS offset cannot be compensated
            return
        lat = msg.latitude
        lon = msg.longitude
        pos_x, pos_y, pos_z = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        pos_x += self.gps_offset * math.cos(self.cur_rot[2])
        pos_y += self.gps_offset * math.sin(self.cur_rot[2])
        self.cur_pos = numpy.array([pos_x, pos_y])
        self.cur_position = numpy.array([pos_x, pos_y, pos_z])
        self.cur_rotation = quaternion_from_euler(self.cur_rot[0], self.cur_rot[1], self.cur_rot[2])
        self.get_logger().info(f" 2D: {self.cur_pos}    3D: {self.cur_position} ROt: {self.cur_rotation}")
        self.get_logger().info(f" x,y,z {pos_x} {pos_y} {pos_z}")

    def imu_callback(self, msg):
        self.cur_rot = euler_from_quaternion(msg.orientation)

    def goal_callback(self, msg):
        # Transform goal pose from GPS to ENU frame of reference
        lat = msg.pose.position.x
        lon = msg.pose.position.y
        pos_x, pos_y, pos_z = gps_to_enu(lat, lon)
        self.cmd_rot = euler_from_quaternion(msg.pose.orientation)
        self.cmd_pos = numpy.array([pos_x, pos_y])
        self.cmd_position = numpy.array([pos_x, pos_y, pos_z])
        self.cmd_rotation = quaternion_from_euler(self.cmd_rot[0], self.cmd_rot[1], self.cmd_rot[2])
        self.get_logger().info(f"cmdpos {self.cmd_pos}  3dcmd {self.cmd_position} ROTcmd {self.cmd_rotation}")

def main(args=None):
    rclpy.init(args=args)

    try:
        usv = StationKeeping()
        rclpy.spin(usv)
    except rclpy.exceptions.ROSInterruptException:
        pass

    usv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

