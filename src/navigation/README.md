### Run the Map Server
rosrun map_server map_server ~/robot_assignment_ws/src/navigation/maps/map.yaml 

### Run the AMCL Node
roslaunch turtlebot_navigation amcl_demo.launch map_file:=~/robot_assignment_ws/src/navigation/maps/map.yaml 

### Run the Navigation Node
rosrun navigation navigate.py

### Provide Goal Coordinates:
Enter the x and y coordinates when prompted by the navigation node.


If you forgot to source the setup.bash script or if you're encountering an error related to a package not being found in ROS after sourcing, you can manually export the ROS_PACKAGE_PATH in your current terminal session *TEMPORARY SOLUTION*:

export ROS_PACKAGE_PATH=~/robot_assignment_ws/src:$ROS_PACKAGE_PATH
