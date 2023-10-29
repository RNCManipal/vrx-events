#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32  # Change Float32 to Float64
from geometry_msgs.msg import PoseStamped

class MoveUSV(Node):
    def __init__(self):
        super().__init__('move_usv')
        self.started_receiving_goal = False
        self.started_receiving_pose_error = False
        self.goal_pose_error_threshold = 1.33
        
        self.curr_imu_x = 0.0
        self.curr_imu_y = 0.0
        self.curr_imu_z = 0.0
        self.curr_imu_w = 0.0

        self.Goal_Sub = self.create_subscription(
            PoseStamped,
            '/vrx/stationkeeping/goal',
            self.goal_callback,
            10
        )

        self.PoseError_Sub = self.create_subscription(
            Float32,  # Change Float32 to Float64
            '/vrx/stationkeeping/pose_error',
            self.pose_error_callback,
            10
        )

        self.thrust_middle_cmd_pub = self.create_publisher(
            Float64,  # Change Float32 to Float64
            '/wamv/thrusters/middle/thrust',
            1
        )

        self.thrust_right_cmd_pub = self.create_publisher(
            Float64,  # Change Float32 to Float64
            '/wamv/thrusters/right/thrust',
            1
        )

        self.thrust_left_cmd_pub = self.create_publisher(
            Float64,  # Change Float32 to Float64
            '/wamv/thrusters/left/thrust',
            1
        )
        
        

        self.rate = self.create_rate(10)  # Adjust the rate as needed

    def goal_callback(self, msg):
        self.started_receiving_goal = True
        
    

    def pose_error_callback(self, msg):
        self.started_receiving_pose_error = True
        pose_error = msg.data

        if pose_error > self.goal_pose_error_threshold:
            self.move_usv_forward()
        else:
            self.stop_usv()
            self.rotate_usv()
            if pose_error < 0.65:
                self.stop_usv()

    def move_usv_forward(self):
        thrust_value = 20.0  # Thrust value to move the USV forward
        msg = Float64()
        msg.data = thrust_value
        self.thrust_middle_cmd_pub.publish(msg)
        self.thrust_right_cmd_pub.publish(msg)
        self.thrust_left_cmd_pub.publish(msg)

    def stop_usv(self):
        # To stop the USV, set thrust value to zero
        msg = Float64()
        msg.data = 0.0
        self.thrust_middle_cmd_pub.publish(msg)
        self.thrust_right_cmd_pub.publish(msg)
        self.thrust_left_cmd_pub.publish(msg)

    def rotate_usv(self):
        val = 30.0
        msg_left = Float64()
        msg_left.data = -val
        msg_mid = Float64()
        msg_mid.data = 1.0
        msg_r = Float64()
        msg_r.data = val
        
        self.thrust_right_cmd_pub.publish(msg_left)
        self.thrust_left_cmd_pub.publish(msg_r)

def main(args=None):
    rclpy.init(args=args)

    try:
        usv = MoveUSV()
        rclpy.spin(usv)
    except rclpy.exceptions.ROSInterruptException:
        pass

    usv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

