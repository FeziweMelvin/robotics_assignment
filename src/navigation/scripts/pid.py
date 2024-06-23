#!/usr/bin/env python
from math import pow, sqrt

import numpy as np
from tf.transformations import euler_from_quaternion


class PID(object):
    def __init__(self) -> None:
        self.kp = 0.5  # Proportional gain
        self.ki = 0.001  # Integral gain
        self.kd = 0.05  # Derivative gain
        self.error = 0  # Current error
        self.prev_error = 0  # Previous error
        self.integral_error = 0  # Integral of error over time
        self.derivative_error = 0  # Rate of change of error
        self.output = 0  # PID output
        self.ang = np.zeros(3)  # Placeholder for angular values

    def get_euclidean_distance(self, state, goal_position) -> float:
        """
        Calculate the Euclidean distance between the robot's current position and the goal position.

        Args:
        - state: Current state of the robot (object with pose information).
        - goal_position: Desired goal position to reach.

        Returns:
        - Euclidean distance between current position and goal position.
        """
        return sqrt(
            pow((goal_position.x - state.pose.position.x), 2)
            + pow((goal_position.y - state.pose.position.y), 2)
        )

    def get_rotation(self, state):
        """
        Extract the yaw (rotation around the z-axis) from the robot's orientation quaternion.

        Args:
        - state: Current state of the robot (object with orientation information).

        Returns:
        - Yaw angle in radians.
        """
        orientation = state.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def compute_pid(self, error):
        """
        Compute the PID output based on the given error (difference between desired and current state).

        Args:
        - error: Current error (difference between desired and current state).

        Returns:
        - PID output based on proportional, integral, and derivative components.
        """
        self.error = error
        # Integral term
        self.integral_error += self.error
        # Derivative term
        self.derivative_error = self.error - self.prev_error
        # PID output calculation
        self.output = (
            self.kp * self.error
            + self.ki * self.integral_error
            + self.kd * self.derivative_error
        )
        # update the previous error
        self.prev_error = self.error

        return self.output
