#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from transformations import euler_from_quaternion
#from geographic_msgs.msg import GeoPoint

class MoveThrusters(Node):

    def __init__(self):
        super().__init__('move_thrusters')

        self.started_recieving_1 = False
        self.started_recieving_2 = False
        self.started_recieving_3 = False
        self.started_recieving_4 = False
        self.started_recieving_5 = False
        self.started_recieving_6 = False

        self.goal_pose_lat = 0.0
        self.goal_pose_lon = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_z = 0.0
        self.goal_pose_w=0.0

        self.curr_gps_lat = 0.0
        self.curr_gps_lon = 0.0

        self.curr_imu_x = 0.0
        self.curr_imu_y = 0.0
        self.curr_imu_z = 0.0
        self.curr_imu_w = 0.0

        self.pose_error = 0.0
        self.dis = 0.0
        
        self.tf_buffer = Buffer()

        self.Goal_Sub = self.create_subscription(
            PoseStamped,
            '/vrx/stationkeeping/goal',
            self.goalcallback,
            10
        )

        self.Pose_error_Sub = self.create_subscription(
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
        )

        self.gps_Sub = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gpscallback,
            10
        )

        self.imu_Sub = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imucallback,
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

        self.rate = self.create_rate(5)

    def goalcallback(self, msg_sub_1: PoseStamped):
        self.started_recieving_1 = True
        self.goal_pose_x = msg_sub_1.pose.position.x
        self.goal_pose_y = msg_sub_1.pose.position.y
        self.goal_pose_z = msg_sub_1.pose.position.z
        self.goal_pose_w=msg_sub_1.pose.orientation.w

    def pecallback(self, msg_sub_2: Float32):
        self.started_recieving_2 = True
        self.pose_error = msg_sub_2.data
        self.get_logger().info("pose error data: %f", self.pose_error)

    def mpecallback(self, msg_sub_3: Float32):
        self.started_recieving_3 = True
        self.mpe = msg_sub_3.data
        self.get_logger().info("mean pose error data: %f", self.mpe)

    def gpscallback(self, msg_gps: NavSatFix):
        self.started_recieving_4 = True
        self.curr_gps_lat = msg_gps.latitude
        self.curr_gps_lon = msg_gps.longitude

    def imucallback(self, msg_imu: Imu):
        self.started_recieving_5 = True
        self.curr_imu_x = msg_imu.orientation.x
        self.curr_imu_y = msg_imu.orientation.y
        self.curr_imu_z = msg_imu.orientation.z
        self.curr_imu_w = msg_imu.orientation.w

    def check_goal(self):
        while (
            not self.started_recieving_1
            and not self.started_recieving_5
            and not self.started_recieving_2
            and not self.started_recieving_4
        ):
            time.sleep(0.6)
        while not self.curr_imu_w:
            time.sleep(0.5)
        while not self.goal_pose_x:
            time.sleep(0.5)
        while not self.curr_gps_lat:
            time.sleep(0.5)

        msg1 = Float64()
        msg2 = Float64()
        msg3 = Float64()
        msg4 = Float64()
        msg5 = Float64()
        msg6 = Float64()
        time.sleep(0.5)

        if self.pose_error > 1.0:
            straight_movement()

        else:
            exit()

    def straight_movement(self):
    	msg1.data = 0.0
    	self.lateral_thrust_angle_Pub.publish(msg1)
    	msg2.data = 20
    	self.lateral_thrust_cmd_Pub.publish(msg2)
	msg3.data = 0.0
	self.left_thrust_angle_Pub.publish(msg3)
	msg4.data = 20
	self.left_thrust_cmd_Pub.publish(msg4)
	msg5.data = 0.0
	self.right_thrust_angle_Pub.publish(msg5)
	msg6.data = 20
	self.right_thrust_cmd_Pub.publish(msg6)

        


                    

    def rotation_usv(self):
        
        (roll, pitch, self.theta) = euler_from_quaternion(
            [
                self.curr_imu_x,
                self.curr_imu_y,
                self.curr_imu_z,
                self.curr_imu_w
            ]
        )
        (roll, pitch, self.yaw) = euler_from_quaternion(
            [
                self.goal_pose_x,
                self.goal_pose_y,
                self.goal_pose_z,
                self.goal_pose_w,
            ]
        )

        
        dy = self.goal_pose_x
        dx = self.goal_pose_y
        self.a_angle = math.atan2(dy, dx)
        rot_angle2 = (self.a_angle - self.theta)

        

        self.dis = math.sqrt(
            (math.pow(self.goal_pose_x, 2))
            + (math.pow(self.goal_pose_y, 2))
        )

        while True:
            if self.dis < 2.56e-05:
                self.straight_movement()

            elif 0.02 < rot_angle2 < 3.3:
                if (
                    abs(rot_angle2) < 0.104
                    or (3.036 < abs(rot_angle2) < 3.14)
                ):
                    msg1.data = math.pi / 2
                    self.lateral_thrust_angle_Pub.publish(msg1)
                    msg2.data = 0.0
                    self.lateral_thrust_cmd_Pub.publish(msg2)
                    msg3.data = 0.0
                    self.left_thrust_angle_Pub.publish(msg3)
                    msg4.data = 0.0
                    self.left_thrust_cmd_Pub.publish(msg4)
                    msg5.data = 0.0
                    self.right_thrust_angle_Pub.publish(msg5)
                    msg6.data = 0.0
                    self.right_thrust_cmd_Pub.publish(msg6)

                    self.straight_movement()
                    self.count = 1
                    return

                elif (0.11 < abs(rot_angle2) < 0.25) or (
                    2.931 < abs(rot_angle2) < 3.035
                ):
                    msg2.data = 0.0
                    msg4.data = -0.375
                    msg6.data = 0.375
                    msg3.data = 0
                    msg5.data = 0
                    msg1.data = 0.0

                    for msg4.data in range(-1, 0):
                        msg4.data += 0.0375
                        self.left_thrust_cmd_Pub.publish(msg4)

                    for msg6.data in range(1, 0):
                        msg6.data -= 0.0375
                        self.right_thrust_cmd_Pub.publish(msg6)

                    self.lateral_thrust_cmd_Pub.publish(msg2)
                    self.lateral_thrust_angle_Pub.publish(msg1)
                    self.left_thrust_angle_Pub.publish(msg3)
                    self.right_thrust_angle_Pub.publish(msg5)
                    return

                else:
                    msg1.data = 0
                    self.lateral_thrust_angle_Pub.publish(msg1)
                    msg2.data = 0.0
                    self.lateral_thrust_cmd_Pub.publish(msg2)
                    msg3.data = 0.0
                    self.left_thrust_angle_Pub.publish(msg3)
                    msg4.data = -0.375
                    self.left_thrust_cmd_Pub.publish(msg4)
                    msg5.data = 0.0
                    self.right_thrust_angle_Pub.publish(msg5)
                    msg6.data = 0.375
                    self.right_thrust_cmd_Pub.publish(msg6)
                    return

            else:
                msg1.data = 0.0
                self.lateral_thrust_angle_Pub.publish(msg1)
                msg2.data = 0.0
                self.lateral_thrust_cmd_Pub.publish(msg2)
                msg3.data = 0.0
                self.left_thrust_angle_Pub.publish(msg3)
                msg4.data = 0.375
                self.left_thrust_cmd_Pub.publish(msg4)
                msg5.data = 0.0
                self.right_thrust_angle_Pub.publish(msg5)
                msg6.data = -0.375
                self.right_thrust_cmd_Pub.publish(msg6)

            return


def main(args=None):
    rclpy.init(args=args)

    try:
        uav = MoveThrusters()
        uav.check_goal()
        
        rclpy.spin(uav)

    except rclpy.exceptions.ROSInterruptException:
        pass

    uav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
