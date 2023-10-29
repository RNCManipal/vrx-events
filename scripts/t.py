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
from transformations import euler_from_quaternion, quaternion_from_euler
from rcl_interfaces.srv import ListParameters, DescribeParameters, GetParameters, SetParameters
import time

thruster_positions = {
    "center_thruster": {
        "position": [0.0, 0.0, 0.318237],
        "orientation": [0.0, 0.0, 0.0]
    },
    "left_thruster": {
        "position": [-2.373776, 1.027135, 0.318237],
        "orientation": [0.0, 0.0, 0.0]
    },
    "right_thruster": {
        "position": [-2.373776, -1.027135, 0.318237],
        "orientation": [0.0, 0.0, 0.0]
    }
}        


config={ "gps_offset":0.85,
        
         "G2G_kP":22.0,
         "G2G_kI":0.27,
         "G2G_kD":2.4,
         "G2G_kS":50,

         "SK_Vx_kP":1.9,
         "SK_Vx_kI":0.12,
         "SK_Vx_kD":1.8,
         "SK_Vx_kS":50,

         "SK_Vy_kP":2.2,
         "SK_Vy_kI":0.12,
         "SK_Vy_kD":1.9,
         "SK_Vy_kS":50,

         "SK_Wz_kP":2.0,
         "SK_Wz_kI":0.12,
         "SK_Wz_kD":2.0,
         "SK_Wz_kS":50,

         "goal_tol":0.3,
         "v_const":5.7,
         "v_limit":2.23,
         "debug":True
}

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

    return east, north, up
	
# 	x = lat0
# 	y = lon0
# 	z = enu[2]
# 	return x, y, z

def smoothen_val(err_rot):
    err_rot = -1 * (err_rot - math.pi)
    if (err_rot>math.pi):
        err_rot = err_rot - (2 * math.pi)
    return err_rot




# last updated oct 16 20:37 all the callbacks working


class StationKeeping(Node):
    def __init__(self):
        super().__init__('thrusters')
        # Initialize station-keeping
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

        
        self.gps_offset = 0.85
        
        self.tf_buffer = Buffer()
        
        
        
        
        self.Mean_pose_error_Sub = self.create_subscription(
            Float32,
            '/vrx/stationkeeping/mean_pose_error',
            self.mpe_callback,
            10
        )

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
        
        self.Pose_error_Sub = self.create_subscription(
            Float32,
            '/vrx/stationkeeping/pose_error',
            self.station_keeping,
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
        # if self.roll is None: # If no yaw data is available, GPS offset cannot be compensated
        #    return
        lat = msg.latitude
        lon = msg.longitude
        pos_x, pos_y, pos_z = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        # pos_x +=  math.cos(self.yaw)
        # pos_y += math.sin(self.yaw)
        self.cur_pos = numpy.array([pos_x, pos_y])
        self.cur_position = numpy.array([pos_x, pos_y, pos_z])
        self.cur_rotation = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        # self.get_logger().info(f" 2D: {self.cur_pos}  ")
        # self.get_logger().info(f" x,y,z {pos_x} {pos_y} {pos_z}")




    def imu_callback(self, msg):

        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        try:
            euler_angles = euler_from_quaternion(quaternion)
            self.roll = euler_angles[2]
            self.pitch = euler_angles[1]
            self.yaw = euler_angles[0]
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
        # err=self.cmd_pos-self.cur_pos
        # total_err = err[0] + 0.75 * err[1]
        # self.time = self.get_clock().now().nanoseconds
        # self.get_logger().info(f"cmdpos {self.cmd_pos}  ")
        # self.get_logger().info(f"Error at time {self.time}: {err}")

    
    def mpe_callback(self,msg):
        meanpose_err = msg.data
        # self.get_logger().info(f" mean pose Error : {meanpose_err}")

    def config_callback(self):
        # Handle updated configuration values
        self.gps_offset = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.pid_g2g    = PIDController(config['G2G_kP'], config['G2G_kI'], config['G2G_kD'], config['G2G_kS']) # Go-to-goal PID controller
        self.pid_sk_vx  = PIDController(config['SK_Vx_kP'], config['SK_Vx_kI'], config['SK_Vx_kD'], config['SK_Vx_kS']) # Station-keeping Vx PID controller
        self.pid_sk_vy  = PIDController(config['SK_Vy_kP'], config['SK_Vy_kI'], config['SK_Vy_kD'], config['SK_Vy_kS']) # Station-keeping Vy PID controller
        self.pid_sk_wz  = PIDController(config['SK_Wz_kP'], config['SK_Wz_kI'], config['SK_Wz_kD'], config['SK_Wz_kS']) # Station-keeping Wz PID controller
        self.goal_tol   = config['goal_tol'] # Goal tolerance dead-band of go-to-goal PID controller
        self.v_const    = config['v_const'] # Proportional gain for linear velocity outside goal tolerance
        self.v_limit    = config['v_limit'] # Saturation limit for linear velocity outside goal tolerance
        self.debug      = config['debug'] # Flag to enable/disable debug messages
        self.config     = config
        return config

    def station_keeping(self, msg):

        if msg.data>0.3:
        # Pose error is larger than a threshold
            self.config = self.config_callback()
            thrustval = [0.0, 0.0, 0.0]
            err_pos = self.cmd_pos - self.cur_pos
            err_pos[1] = err_pos[1]*8.9

            # Current time
            self.time = self.get_clock().now().nanoseconds

            # err_pos = self.cmd_pos - self.cur_pos # Error in position [x_des - x_cur, y_des - y_cur]
            # if numpy.linalg.norm(err_pos) > self.goal_tol: # (Euclidean distance to goal as L2 norm)
                # If far from goal, head directly towards the goal by controlling Vx & Wz
            # self.get_logger().info(f"checkpoint 1")
            
            V_x = numpy.linalg.norm(err_pos) * self.v_const # P controller for Vx

            if V_x > self.v_limit: # Clamp linear velocity along X-axis
                V_x = self.v_limit
            V_y = 0.0

            err_rot = (normalize_angle((-1 * math.atan2(err_pos[1],err_pos[0])) - self.yaw))#This values fluctuates between very large and small values quickly
            err_rot = smoothen_val(err_rot)

            W_z = self.pid_g2g.control(err_rot, self.time) # PID controller for Wz


            self.distance_between_thrusters = 2.05427
            self.thruster_max_thrust = 200
            
            left_thrust = ((2 * V_x) -  (W_z * self.distance_between_thrusters))/(2 * self.thruster_max_thrust)
            right_thrust = ((2 * V_x) + (W_z * self.distance_between_thrusters))/(2 * self.thruster_max_thrust)

            self.get_logger().info(f"err rot: {err_rot} ")
            self.get_logger().info(f"rot Vel: {W_z} ")
            self.get_logger().info(f"err pos: {numpy.linalg.norm(err_pos)}")
            self.get_logger().info(f"Pos Vel: {V_x} ")
            self.get_logger().info(f"le {left_thrust} re {right_thrust}")



            thrustval[0] = 0
            thrustval[1] = left_thrust
            thrustval[2] = right_thrust

            thrust_m = Float64()
            thrust_m.data = numpy.float64(thrustval[0])
            thrust_l = Float64()
            thrust_l.data = numpy.float64(thrustval[1])
            thrust_r = Float64()
            thrust_r.data = numpy.float64(thrustval[2])

            self.lateral_thrust_cmd_Pub.publish(thrust_m)
            self.left_thrust_cmd_Pub.publish(thrust_l)
            self.right_thrust_cmd_Pub.publish(thrust_r)


        '''else:
            thrustval[0] = 0
            thrustval[1] = 0
            thrustval[2] = 0

            thrust_m = Float64()
            thrust_m.data = numpy.float64(thrustval[0])
            thrust_l = Float64()
            thrust_l.data = numpy.float64(thrustval[1])
            thrust_r = Float64()
            thrust_r.data = numpy.float64(thrustval[2])

            self.lateral_thrust_cmd_Pub.publish(thrust_m)
            self.left_thrust_cmd_Pub.publish(thrust_l)
            self.right_thrust_cmd_Pub.publish(thrust_r)'''
        
        
        '''# If near the goal, perform fine adjustments in Vx, Vy, and Wz for station-keeping
        rot_tf = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - self.yaw)  # G2G rotation transformation
        err_pos = numpy.array([numpy.linalg.norm(err_pos) * math.cos(rot_tf), numpy.linalg.norm(err_pos) * math.sin(rot_tf)])  # Error in position (in local frame)
        V_x = self.pid_sk_vx.control(err_pos[0], self.time)  # PID controller for Vx
        V_y = self.pid_sk_vy.control(err_pos[1], self.time)  # PID controller for Vy
        err_rot = normalize_angle(self.cmd_rot[0] - self.yaw)  # Error in orientation
        W_z = self.pid_sk_wz.control(err_rot, self.time)
        # W_z = math.atan(W_z)  # PID controller for Wz
        self.get_logger().info(f"wz value {W_z}  V_x {V_x} V_y {V_y}")'''



        '''def calculate_thrust(V_x, V_y, W_z):
            # This is a simple example; you should replace this with your actual thrust calculation model
            # Here, we assume a linear relationship between input velocities and thrust
            thrust = 1 * (V_x + V_y) + self.config['SK_Wz_kS'] * W_z
            return thrust

        # Initialize lists to store thruster commands and angles
        thrustval = [0.0, 0.0, 0.0]
        angleval = [0.0, 0.0, 0.0]  

        # Convert linear and angular velocities to individual thruster command
        for i, (thruster_name, thruster_info) in enumerate(thruster_positions.items()):
            position = numpy.array(thruster_info["position"])
            orientation = numpy.array(thruster_info["orientation"])

            # Calculate the local linear and angular velocity components
            local_V_x = V_x * math.cos(orientation[2]) + V_y * math.sin(orientation[2])
            local_V_y = -V_x * math.sin(orientation[2]) + V_y * math.cos(orientation[2])
            local_W_z = W_z

            # Calculate thrust using your logic
            thrust = calculate_thrust(local_V_x, local_V_y, local_W_z)

            # Calculate the angle for the thruster
            angle = math.atan2(local_V_y, local_V_x)

            # Store the thruster command in the lists
            thrustval[i] = thrust
            angleval[i] = angle

        # Publish thruster commands and angles
        thrust_m = Float64()
        thrust_m.data = numpy.float64(thrustval[0])
        thrust_l = Float64()
        thrust_l.data = numpy.float64(thrustval[1])
        thrust_r = Float64()
        thrust_r.data = numpy.float64(thrustval[2])

        angle_m = Float64()
        if angleval[0] >= math.pi/2:
            angle_m.data = math.pi - angleval[0]
        angle_m.data = -angleval[0]

        angle_l = Float64()
        if angleval[1] >= math.pi/2:
            angle_l.data = math.pi - angleval[1]
        angle_l.data = -angleval[1]
        
        angle_r = Float64()
        if angleval[2] >= math.pi/2:
            angle_r.data = math.pi - angleval[2]
        angle_r.data = -angleval[2]
        # Publish thruster commands and angles as needed
        self.lateral_thrust_cmd_Pub.publish(thrust_m)
        self.left_thrust_cmd_Pub.publish(thrust_l)
        self.right_thrust_cmd_Pub.publish(thrust_r)
        self.lateral_thrust_angle_Pub.publish(angle_m)
        self.left_thrust_angle_Pub.publish(angle_l)
        self.right_thrust_angle_Pub.publish(angle_r)

    else:
        # Pose error is less than the threshold, stop thrusters
        thrust_m = Float64()
        thrust_l = Float64()
        thrust_r = Float64()
        angle_m = Float64()
        angle_l = Float64()
        angle_r = Float64()

        thrust_m.data = 0.0
        thrust_l.data = 0.0
        thrust_r.data=0.0
        angle_m.data=0.0
        angle_l.data=0.0
        angle_r.data=0.0



        self.lateral_thrust_cmd_Pub.publish(thrust_m)
        self.left_thrust_cmd_Pub.publish(thrust_l)
        self.right_thrust_cmd_Pub.publish(thrust_r)
        self.lateral_thrust_angle_Pub.publish(angle_m)
        self.left_thrust_angle_Pub.publish(angle_l)
        self.right_thrust_angle_Pub.publish(angle_r)'''





'''def station_keeping(self,msg):
    self.get_logger().info(f"pose error {msg.data}")
    if msg.data>0.5:
        self.get_logger().info(f"enetred")
        self.config = self.config_callback()
        err_pos = self.cmd_pos - self.cur_pos # Error in position [x_des - x_cur, y_des - y_cur]
        self.time = self.get_clock().now().nanoseconds
        # TAKE GOAL_TOL, .....
        if numpy.linalg.norm(err_pos) > self.goal_tol:

            # If far: Vx & Wz
            V_x = numpy.linalg.norm(err_pos) * self.v_const # P controller for Vx
            if V_x > self.v_limit: # Clamp linear velocity along X-axis
                V_x = self.v_limit
            V_y = 0.0
            err_rot = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - self.roll) # Error in orientation
            W_z = self.pid_g2g.control(err_rot, self.time) # PID controller for Wz
        else:
            # If near goal, perform fine adjustments in Vx, Vy & Wz for station-keeping
            rot_tf = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - self.roll) # G2G rotation transformation
            err_pos = numpy.array([numpy.linalg.norm(err_pos) * math.cos(rot_tf), numpy.linalg.norm(err_pos) * math.sin(rot_tf)]) # Error in position (in local frame)
            V_x = self.pid_sk_vx.control(err_pos[0], self.time) # PID controller for Vx
            V_y = self.pid_sk_vy.control(err_pos[1], self.time) # PID controller for Vy
            err_rot = normalize_angle(self.cmd_rot[2] - self.roll) # Error in orientation
            W_z = self.pid_sk_wz.control(err_rot, self.time) # PID controller for Wz
        # print("Station-Keeping Coordinates: {:.4} m, {:.4} m, {:.4} rad".format(self.cmd_pos[0], self.cmd_pos[1], self.cmd_rot[2]))
        # self.get_logger().info(f"Vx, Vy, Wz: {V_x}, {V_y}, {W_z}")
        

        thrust_mid = (V_x**2 + V_y**2)**0.5
        thrust_left = thrust_mid - (V_x - V_y)
        thrust_right = thrust_mid + (V_x - V_y)
        d = 1.775071
        angle_left = math.atan2(-W_z * d / (2 * V_x + 1e-6), 1.0)
        angle_right = math.atan2(W_z * d / (2 * V_x + 1e-6), 1.0)

        self.get_logger().info(f"the r angle value {angle_right} the l value {angle_left}")

        

        def calculate_thrust(V_x, V_y, W_z):
                # This is a simple example; you should replace this with your actual thrust calculation model
                # # Here, we assume a linear relationship between input velocities and thrust
            thrust = 1 * (V_x + V_y) + config['SK_Wz_kS'] * W_z
            return thrust

        
            # Initialize an empty dictionary to store thruster commands
        
        thrustval = [0.0,0.0,0.0]
        angleval = [0.0,0.0,0.0]
        i = 0
            #        Convert linear and angular velocities to individual thruster command
        for thruster_name, thruster_info in thruster_positions.items():
            position = numpy.array(thruster_info["position"])
            orientation = numpy.array(thruster_info["orientation"])
            # Calculate the local linear and angular velocity components
            local_V_x = V_x * math.cos(orientation[2]) + V_y * math.sin(orientation[2])
            local_V_y = -V_x * math.sin(orientation[2]) + V_y * math.cos(orientation[2])
            local_W_z = W_z
            # Your thrust calculation logic here, e.g., linear relationship
            thrust = calculate_thrust(local_V_x, local_V_y, local_W_z)
            # Calculate the angle for the thruster
            angle = math.atan2(local_V_y, local_V_x)
            # Store the thruster command in the dictionary
            thrustval[i] = thrust
            angleval[i] = angle
            i = i+1            
        
            
        thrust_m = Float64()
        thrust_m.data = numpy.float64(thrustval[0])
        thrust_l = Float64()
        thrust_l.data = numpy.float64(thrustval[1])
        thrust_r = Float64()
        thrust_r.data = numpy.float64(thrustval[2])
        
        angle_m = Float64()
        angle_m.data = -angleval[0]
        angle_l = Float64()
        angle_l.data = -angleval[1]
        angle_r = Float64()
        angle_r.data = -angleval[2]





        # self.lateral_thrust_angle_Pub.publish(angle_l)
        self.lateral_thrust_cmd_Pub.publish(thrust_m)
        self.left_thrust_cmd_Pub.publish(thrust_l)
        self.right_thrust_cmd_Pub.publish(thrust_r)
        self.left_thrust_angle_Pub.publish(angle_l)
        self.right_thrust_angle_Pub.publish(angle_r)
        self.lateral_thrust_angle_Pub.publish(angle_m)
        
        
        

    else:
        self.lateral_thrust_cmd_Pub.publish(0)
        self.left_thrust_cmd_Pub.publish(0)
        self.right_thrust_cmd_Pub.publish(0)
        self.lateral_thrust_angle_Pub.publish(0)
        self.left_thrust_angle_Pub.publish(0)
        self.right_thrust_angle_Pub.publish(0)'''


         
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
