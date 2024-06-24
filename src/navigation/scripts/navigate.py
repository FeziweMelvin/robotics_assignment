#!/usr/bin/env python

import prm
import rospy
from pid import PID
from math import atan2
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from turtlesim.msg import Pose


class TurtleBot:
    def __init__(self):
        # Initialize the ROS node for the TurtleBot controller
        rospy.init_node("turtlebot_controller", anonymous=True)

        # Create a publisher for the robot's velocity commands
        self.velocity_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

        # Create a subscriber to update the robot's position from the Gazebo simulator
        self.pose_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.update_position)

        # Initialize the robot's pose (position and orientation)
        self.pose = Pose()

        # Set the rate at which the loop will run (10 Hz)
        self.rate = rospy.Rate(10)

    def update_position(self, msg):
        index = 0
        # Iterate through the list of model names in the received message
        for i in range(len(msg.name)):
            if msg.name[i] == "mobile_base":
                index: int = i
                break

        # Extract the x and y positions of the robot from the message
        x = msg.pose[index].position.x
        y = msg.pose[index].position.y
        # Extract the orientation quaternion of the robot from the message
        rot_q = msg.pose[index].orientation

        _, _, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        # Update the robot's pose with the new position and orientation
        self.pose.x = round(x, 4)
        self.pose.y = round(y, 4)
        self.pose.theta = theta

    # def update_position(self, msg) -> None:
    #     # Find the index of "mobile_base" in the list of model names
    #     try:
    #         index = msg.name.index("mobile_base")
    #     except ValueError:
    #         rospy.logwarn("Cannot find 'mobile_base' in model states message.")
    #         return

    #     # Extract position and orientation data
    #     x = msg.pose[index].position.x
    #     y = msg.pose[index].position.y
    #     rot_q = msg.pose[index].orientation

    #     # Convert quaternion to Euler angles (roll, pitch, yaw)
    #     _, _, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    #     # Update the robot's pose attributes
    #     self.pose.x = round(x, 4)
    #     self.pose.y = round(y, 4)
    #     self.pose.theta = theta


    @staticmethod
    def get_current_state():
        # Wait for the service "/gazebo/get_model_state" to be available
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            # Create a service proxy to call the service
            gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            # Call the service to get the state of the "mobile_base" model
            state = gms(model_name="mobile_base")
            return state
        except rospy.ServiceException as e:
            # Print an error message if the service call fails
            print("Service call failed: " + str(e))

    def steering_angle(self, state, goal_position) -> float:
        # Calculate the difference in the y-coordinates between the goal position and the current state
        delta_y = goal_position.y - state.pose.position.y
        # Calculate the difference in the x-coordinates between the goal position and the current state
        delta_x = goal_position.x - state.pose.position.x
        # Calculate and return the steering angle using the arctangent of delta_y / delta_x
        # atan2 handles the quadrant-specific calculation to return the correct angle
        return atan2(delta_y, delta_x)

    def angular_vel(self, state, theta, goal_pose, constant=3):
        # Calculate the steering angle towards the goal position
        ang_vel = constant * (self.steering_angle(state, goal_pose) - theta)
        # Calculate the required angular velocity
        # The constant determines how aggressively the robot will turn towards the target angle
        max_vel = 7
        # Check if the calculated angular velocity is within the allowable range
        if abs(ang_vel) < max_vel:
            return ang_vel
        else:
            if ang_vel < 0:
                return -max_vel
            else:
                return max_vel
                

    def steer(self, goal_position) -> None:
        # Initialize a Twist message for velocity control
        vel_msg = Twist()

        # Loop until ROS is shutdown
        while not rospy.is_shutdown():
            # Create a PID controller instance
            pid_controller = PID()
            # Get the current state of the robot
            state = self.get_current_state()
            # Calculate the current orientation of the robot
            theta = pid_controller.get_rotation(state)
            # Compute the PID output for distance control
            pid_dist = pid_controller.compute_pid(
                pid_controller.get_euclidean_distance(state, goal_position)
            )

            vel_msg.linear.x = pid_dist

            # Ensure no rotational movement in x and y axes
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(state, theta, goal_position)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Check if the robot is close enough to the goal position
            if pid_controller.get_euclidean_distance(state, goal_position) < 0.05:
                # the goal has been found
                break

    def follow_path(self, path) -> None:
        goal_pose = Pose()
        # Iterate through each step in the given path
        for step in path:
            # Set the goal position (x, y) from the current step in the path
            goal_pose.x = step[0]
            goal_pose.y = step[1]
            # Navigate towards the current goal position using the steer method
            self.steer(goal_pose)


if __name__ == "__main__":
    try:
        path = prm.main()
        path = path[::-1]
        x = TurtleBot()
        x.follow_path(path)

    except rospy.ROSInterruptException:
        pass
