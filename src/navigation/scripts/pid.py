#!/usr/bin/env python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

class PID(object):
    def __init__(self):
        self.kp = 0.5
        self.ki = 0.001
        self.kd = 0.05
        self.error = 0
        self.prev_error = 0 
        self.integral_error = 0 
        self.derivative_error = 0 
        self.output = 0
        self.ang = np.zeros(3)
        
    def get_euclidean_distance(self, state, goal_position):
        return sqrt(pow((goal_position.x - state.pose.position.x), 2) +
                    pow((goal_position.y - state.pose.position.y), 2))
    
    def get_rotation(self, state):           
        orientation = state.pose.orientation 
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw
 	
    def compute_pid(self, error):
        self.error = error 
        
        self.integral_error += self.error
        self.derivative_error = self.error - self.prev_error

        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
        # update the previous error
        self.prev_error = self.error

        return self.output