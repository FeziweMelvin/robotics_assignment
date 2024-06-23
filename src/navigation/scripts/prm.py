#!/usr/bin/env python
import math
import os
import sys

import numpy as np
from control import TurtleBot
from scipy.spatial import KDTree


class Node:
    def __init__(self, x, y, cost, parent_index) -> None:
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self) -> str:
        return str(self.x) + "," + str(self.y) + "," + str(self.cost)


def prm(sx, sy, gx, gy, obstacles_x_values, obstacles_y_values, robot_radius):
    # Create a KDTree for efficient nearest neighbor search with obstacle coordinates
    obstables = KDTree(np.vstack((obstacles_x_values, obstacles_y_values)).T)

    # Generate random points within the map, avoiding obstacles
    points_x_values, points_y_values = generate_random_points(
        sx, sy, gx, gy, robot_radius, obstacles_x_values, obstacles_y_values, obstables
    )

    # Create a roadmap graph connecting points based on collision-free paths
    road_map = generate_map(points_x_values, points_y_values, robot_radius, obstables)

    # Use Dijkstra's algorithm to find the shortest path from start to goal
    px, py = dijkstra_planning(
        sx, sy, gx, gy, road_map, points_x_values, points_y_values
    )

    # Return the x and y coordinates of the path
    return px, py


def is_collision(sx, sy, gx, gy, rr, NN) -> bool:
    # Calculate the Euclidean distance between the start (sx, sy) and goal (gx, gy) points
    d: float = math.hypot(gx - sx, gy - sy)

    # If the distance is greater than or equal to 30 m, assume a collision (for performance or safety reasons)
    if d >= 30:
        return True

    # Calculate the number of intermediate points to check along the line from start to goal
    n = int(d / rr)
    # Iterate over the intermediate points
    for _ in range(n):
        dist, _ = NN.query([sx, sy])
        # If the nearest obstacle is within the robot's radius, a collision is detected
        if dist <= rr:
            return True
        # Move to the next intermediate point along the line towards the goal
        sx = sx + rr * math.cos(math.atan2(gy - sy, gx - sx))
        sy = sy + rr * math.sin(math.atan2(gy - sy, gx - sx))

    # Check the distance from the goal point to the nearest obstacle
    dist, _ = NN.query([gx, gy])

    # If the nearest obstacle to the goal is within the robot's radius, a collision is detected
    if dist <= rr:
        return True

    return False


def generate_map(random_x_values, random_y_values, rr, obstacles):
    # Initialize the roadmap as an empty list
    map = []
    # Number of random points to sample
    n: int = len(random_x_values)
    # Create a KDTree for efficient nearest neighbor search
    random_tree = KDTree(np.vstack((random_x_values, random_y_values)).T)

    # Iterate through each random point
    for _, rx, ry in zip(range(n), random_x_values, random_y_values):
        # Find the nearest neighbors to the current point
        _, indexes = random_tree.query([rx, ry], k=n)
        edge_id = []

        # Iterate through the nearest neighbors for  x & y-coordinate of the neighbor
        for i in range(1, len(indexes)):
            nx = random_x_values[indexes[i]]
            ny = random_y_values[indexes[i]]

            # Check if the path from the current point to the neighbor is collision-free and add the neighbor to the list of edges
            if not is_collision(rx, ry, nx, ny, rr, obstacles):
                edge_id.append(indexes[i])
            # Limit the number of edges per node to 10
            if len(edge_id) >= 10:
                break
        # Add the list of edges to the roadmap
        map.append(edge_id)
    # Return the generated roadmap to be used by dijkstra
    return map


def dijkstra_planning(sx, sy, gx, gy, road_map, randomx, randomy):
    # Initialize the cost to zero and create the start and goal nodes
    cost = 0
    start_node = Node(sx, sy, cost, -1)
    goal_node = Node(gx, gy, cost, -1)

    # Dictionaries to keep track of nodes that are open (to be explored) and closed (already explored)
    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True
    while True:
        # If there are no more nodes to explore, the path cannot be found
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        # Get the node with the smallest cost from the open set
        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

        # If the current node is the goal, path is found
        if c_id == (len(road_map) - 1):
            print("Goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Move the current node from the open set to the closed set
        del open_set[c_id]
        closed_set[c_id] = current

        # Explore neighbors of the current node
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = randomx[n_id] - current.x
            dy = randomy[n_id] - current.y

            # Calculate the distance to the neighbor
            d = math.hypot(dx, dy)
            node = Node(randomx[n_id], randomy[n_id], current.cost + d, c_id)

            # Skip the neighbor if it's already in the closed set
            if n_id in closed_set:
                continue

            # If the neighbor is in the open set, check if this path is cheaper
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    # If no path was found, return empty lists
    if path_found is False:
        return [], []

    # Backtrack from the goal to the start to find the path
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


# Function to Sample random points in the free space of the occupancy grid
def generate_random_points(sx, sy, gx, gy, rr, ox, oy, NN):
    random_x_values = []
    random_y_values = []

    max_x = max(ox)
    max_y = max(oy)
    min_x = min(ox)
    min_y = min(oy)

    # Generate random values within the borders of the environment
    while len(random_x_values) <= 500:
        x = (np.random.random() * (max_x - min_x)) + min_x
        y = (np.random.random() * (max_y - min_y)) + min_y

        # query phase for roadmap to find paths
        dist, _ = NN.query([x, y])

        if dist >= rr:
            random_x_values.append(x)
            random_y_values.append(y)

    # include start and goal points
    random_x_values.append(sx)
    random_x_values.append(gx)

    random_y_values.append(sy)
    random_y_values.append(gy)

    return random_x_values, random_y_values


def changeToWorldCoords(r, c):
    # Conversion using resolution and origin of the the world coordinates
    x = (r * 0.05) - 20.0759
    y = (c * 0.05) - 20
    return x, y


def changeToPixelCoords(x, y):
    # Conversion using resolution and origin of the the world coordinates
    r = round((x + 20.0759) / 0.05)
    c = round((20 + y) / 0.05)
    return r, c


def changeToFinalCoords(finalpath):
    # change pixel co ords of final path to world co ords
    for i in range(finalpath.shape[0]):
        finalpath[i][0], finalpath[i][1] = changeToWorldCoords(
            round(finalpath[i][0]), round(finalpath[i][1])
        )
    # return final path
    return finalpath


def main():
    print(__file__ + " start.")

    # Get the current position of the robot
    state = TurtleBot.get_current_state()
    start_x = state.pose.position.x
    start_y = state.pose.position.y

    # get goal coordinates from terminal
    goal_x = float(input("Input the goal x value:"))
    goal_y = float(input("Input the goal y value:"))

    # have to change to pixel co ords
    start_x, start_y = changeToPixelCoords(start_x, start_y)
    goal_x, goal_y = changeToPixelCoords(goal_x, goal_y)

    print("Start co-ords in pixel co-ords: ", start_x, start_y)
    print("Goal co-ords in pixel co-ords: ", goal_x, goal_y)

    # Assume the robots radius is 5m
    robot_radius = 5.0

    # stores the Occupancy map data into an array
    imgArr = []

    # copy the occupancy map data into the ImgArr
    with open(os.path.join(sys.path[0], "OccupancyMap.txt")) as textFile:
        for line in textFile:
            lines = line.split(",")
            imgArr.append(lines)

    imgArr = np.array(imgArr).astype(int)

    # Arrays to store coordinates of obstacles
    obstacles_x_values = []
    obstacles_y_values = []
    for i in range(imgArr.shape[0]):
        for j in range(imgArr.shape[1]):
            if imgArr[i][j] == 1:  # an obstacle is represented by 1
                obstacles_x_values.append(i)
                obstacles_y_values.append(j)

    px, py = prm(
        start_x,
        start_y,
        goal_x,
        goal_y,
        obstacles_x_values,
        obstacles_y_values,
        robot_radius,
    )

    assert px, "Path not found"

    finalpath = np.column_stack((px, py))

    # change to world co ords
    finalpath = changeToFinalCoords(finalpath)

    return finalpath
