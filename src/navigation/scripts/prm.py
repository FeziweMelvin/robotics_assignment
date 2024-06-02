#!/usr/bin/env python
import sys, os
import math
from scipy.spatial import KDTree
import imp
import numpy as np
from pid import PID
import rospy


class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) 

def prm(sx, sy, gx, gy, obstacles_x_values, obstacles_y_values, robot_radius):
    
    obstables = KDTree(np.vstack((obstacles_x_values, obstacles_y_values)).T)

    points_x_values ,points_y_values = generate_random_points(sx, sy, gx, gy, robot_radius, obstacles_x_values, obstacles_y_values, obstables)

    road_map = generate_map(points_x_values, points_y_values, robot_radius, obstables)
 
    px, py = dijkstra_planning(sx, sy, gx, gy, road_map, points_x_values, points_y_values)
    
    return px, py


def is_collision(sx, sy, gx, gy, rr, NN):
    d = math.hypot(gx - sx, gy - sy)

    if d >= 30:
        return True

    n = int(d / rr)
    for _ in range(n):
        dist, _ = NN.query([sx, sy])
        if dist <= rr:
            return True  
        sx = sx + rr * math.cos(math.atan2(gy - sy, gx - sx))
        sy = sy + rr * math.sin(math.atan2(gy - sy, gx - sx))
    
    dist, _ = NN.query([gx, gy])
    if dist <= rr:
        return True  

    return False  


def generate_map(random_x_values, random_y_values, rr, obstacles):
    map = []
    n = len(random_x_values)
    random_tree = KDTree(np.vstack((random_x_values, random_y_values)).T)

    for (_, rx, ry) in zip(range(n),random_x_values, random_y_values):
        _, indexes = random_tree.query([rx, ry], k=n)
        edge_id = []

        for i in range(1, len(indexes)):
            nx = random_x_values[indexes[i]]
            ny = random_y_values[indexes[i]]

            if not is_collision(rx, ry, nx, ny, rr, obstacles):
                edge_id.append(indexes[i])

            if len(edge_id) >= 10:
                break

        map.append(edge_id)

    return map


def dijkstra_planning(sx, sy, gx, gy, road_map, randomx, randomy):
    cost = 0
    start_node = Node(sx, sy, cost, -1)
    goal_node = Node(gx, gy, cost, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True
    while True:
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break
        
        del open_set[c_id]
        
        closed_set[c_id] = current

        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = randomx[n_id] - current.x
            dy = randomy[n_id] - current.y
            d = math.hypot(dx, dy)
            node = Node(randomx[n_id], randomy[n_id],
                        current.cost + d, c_id)

            if n_id in closed_set:
                continue
          
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    if path_found is False:
        return [], []

   
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


def generate_random_points(sx, sy, gx, gy, rr, ox, oy, NN):
    random_x_values =[]
    random_y_values = []

    max_x = max(ox)
    max_y = max(oy)
    min_x = min(ox)
    min_y = min(oy)

    while len(random_x_values) <= 500:
        x = (np.random.random() * (max_x - min_x)) + min_x
        y = (np.random.random() * (max_y - min_y)) + min_y

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

def changeToWorldCoords(r,c):
    y = 12.84753-(c*0.05188378405)
    x = 6.72835-(r*0.05324720745)
    return x, y

def changeToPixelCoords(x, y):
    c = round((12.84753 - y) / 0.05188378405)
    r = round((6.72835 - x) / 0.05324720745)
    return r, c

def changeToFinalCoords(finalpath):
    # change pixel co ords of final path to world co ords
    for i in range(finalpath.shape[0]):
        finalpath[i][0],finalpath[i][1]=changeToWorldCoords(round(finalpath[i][0]),round(finalpath[i][1]))

    return finalpath


def main():
    print(__file__ + " start.")
    # sx= 4.73219831575
    # sy=4.08815854895
 
    pid = PID()
    state = pid.get_robot_current_state()
    start_x = state.pose.position.x
    start_y = state.pose.position.y

    # get goal coordinates from terminal 
    goal_x = float(input('Input the goal x value:'))
    goal_y = float(input('Input the goal y value:'))
    
    # gx=3.78583484097
    # gy=6.65431327362

    #have to change to pixel co ords
    start_x, start_y = changeToPixelCoords(start_x, start_y)
    goal_x, goal_y = changeToPixelCoords(goal_x, goal_y)

    print("Start co-ords in pixel co-ords: " , start_x ,start_y)
    print("Goal co-ords in pixel co-ords: ", goal_x ,goal_y)

    robot_radius = 5.0

    imgArr = []
    with open(os.path.join(sys.path[0], "mapArray.txt")) as textFile:
        for line in textFile:
            lines=line.split(',')
            imgArr.append(lines)

    imgArr = np.array(imgArr).astype(int)

    obstacles_x_values = []
    obstacles_y_values  = []
    for i in range(imgArr.shape[0]): 
        for j in range(imgArr.shape[1]):
            if imgArr[i][j] == 1:   # an obstacle is represented by 1
                obstacles_x_values.append(i)
                obstacles_y_values .append(j)

    px, py = prm(start_x, start_y, goal_x, goal_y, obstacles_x_values, obstacles_y_values, robot_radius)

    assert px, 'Path not found'

    finalpath = np.column_stack((px,py))
   
    # change to world co ords
    finalpath = changeToFinalCoords(finalpath)
    print(finalpath)

    return finalpath

path = main()
    
