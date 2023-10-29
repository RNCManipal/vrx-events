#!/usr/bin/env python3

import queue
from geometry_msgs.msg import Twist
from rclpy.node import Node

################################################################################

class PIDController(Node):

    '''
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    '''
    def __init__(self, kP, kI, kD, kS):
        super().__init__("pidval")
        self.kP       = kP # Proportional gain
        self.kI       = kI # Integral gain
        self.kD       = kD # Derivative gain
        self.kS       = kS # Saturation constant (error history buffer size)
        self.err_int  = 0 # Error integral
        self.err_dif  = 0 # Error difference
        self.err_prev = 0 # Previous error
        self.err_hist = queue.Queue(self.kS) # Limited buffer of error history
        self.t_prev   = 0 # Previous time

    def control(self, err, t):
        '''
        Generate PID controller output.

        :param err: Instantaneous error in control variable w.r.t. setpoint
        :param t  : Current timestamp

        :return u: PID controller output
        '''
        dt = t - self.t_prev # Timestep
        if dt > 0.0:
            self.err_hist.put(err)
            self.err_int += err
            if self.err_hist.full(): # Jacketing logic to prevent integral windup
                self.err_int -= self.err_hist.get() # Rolling FIFO buffer
            self.err_dif = (err - self.err_prev)
            u = ((self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt))
            # self.get_logger().info(f"PID ERR: {err}")
            # self.get_logger().info(f"PID VAL: {u}")
        
            self.err_prev = err
            self.t_prev = t
            return u # Control signal

################################################################################


