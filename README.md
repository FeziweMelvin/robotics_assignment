# TurtleBot Navigation System

## Description
This project demonstrates a navigation system for TurtleBot using ROS (Robot Operating System). It utilizes PRM (Probabilistic Roadmap), PID (Proportional-Integral-Derivative) controller, and Dijkstra's algorithm for path planning and control.

### Features
The TurtleBot navigation system allows the robot to autonomously navigate from a starting point to a specified goal on a given map. The system utilizes a combination of techniques:

- **Probabilistic Roadmap (PRM):** Generates a roadmap of the environment by sampling random points and connecting feasible paths between them, avoiding obstacles.
  
- **PID Controller:** Controls the movement of the robot by adjusting linear and angular velocities based on the error between the current position and the goal position.
  
- **Dijkstra's Algorithm:** Finds the shortest path on the generated roadmap from the starting point to the goal point, considering the costs associated with each path segment.

The system is implemented in Python and ROS, using packages like OpenCV for image processing and visualization, and Gazebo for simulation.

## Usage

### Installation
1. Clone the repository:
git clone <repository_url>
cd <repository_name>


2. Install dependencies:
pip install scikit-image>=0.15,<0.17


### Running the Code
- Ensure that ROS environment is set up properly and all necessary nodes are running.

1. Launch the ROS environment:
./startWorld

please add more
