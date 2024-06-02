#!/usr/bin/env python
import rospy
import numpy as np
from gazebo_msgs.srv import GetModelState
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

    def get_robot_current_state(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            state = gms(model_name="mobile_base")
            return state
        except rospy.ServiceException as e:
            print('Service call failed: ' + str(e)) 

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