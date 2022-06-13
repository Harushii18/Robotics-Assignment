#!/usr/bin/env python

from calendar import c
import sys, os

import math

# import matplotlib.pyplot as plt
from scipy.spatial import KDTree
# import matplotlib.pyplot as plt
# import cv2
import imp

from math import pow, atan2, sqrt

# from turtlebot_control import Turtlebot

import numpy as np
# from skimage.morphology import binary_erosion, binary_opening, disk, square
# !pip install opencv-python
# !pip install scikit-image

from pid import PID
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, Point
import rospy



class Node:
    #for dijkstra

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) 
               
def changeToWorldCoords(r,c):
    y=12.84753-(c*0.05188378405)
    x=6.72835-(r*0.05324720745)
    return x,y

def changeToPixelCoords(x,y):
    c=round((12.84753-y)/0.05188378405)
    r=round((6.72835-x)/0.05324720745)
    return r,c

def prm(sx, sy, gx, gy,ox, oy, robot_radius):
    
    NN = KDTree(np.vstack((ox, oy)).T)  #For nearest neighbour 

    #generates random points on the map 
    randomx,randomy = random_points(sx, sy, gx, gy,robot_radius,ox, oy,NN)
    #generates map with the random points where the points dont collide with any obstacles 
    road_map = generateMap(randomx, randomy,robot_radius, NN)
    #returns path 
    px, py = dijkstra_planning(sx, sy, gx, gy, road_map, randomx, randomy)

    return px, py


def is_collision(sx, sy, gx, gy, rr, NN):
   
    val = math.hypot(gx - sx, gy - sy)

    if val >= 30.0:
        return True

    n = int(val / rr)

    for i in range(n):
        dist, indexes = NN.query([sx, sy])
        if dist <= rr:
            return True  
        sx = sx + rr * math.cos(math.atan2(gy - sy, gx - sx))
        sy = sy + rr * math.sin(math.atan2(gy - sy, gx - sx))

    
    dist, indexes =NN.query([gx, gy])
    if dist <= rr:
        return True  

    return False  


def generateMap(randomx, randomy, rr,NN):
   

    gmap = []
    n = len(randomx)
    sampleNN = KDTree(np.vstack((randomx, randomy)).T)

    for (r, rx, ry) in zip(range(n),randomx, randomy):

        dists, indexes = sampleNN.query([rx, ry], k=n)
        edge_id = []

        for i in range(1, len(indexes)):
            nx = randomx[indexes[i]]
            ny = randomy[indexes[i]]

            if not is_collision(rx, ry, nx, ny, rr, NN):
                edge_id.append(indexes[i])

            if len(edge_id) >= 10:
                break

        gmap.append(edge_id)


    return gmap


def dijkstra_planning(sx, sy, gx, gy, road_map, randomx, randomy):
    
    cost=0.0
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



def random_points(sx, sy, gx, gy, rr, ox, oy,NN):
    
    #will store random samples 
    randomx =[]
    randomy = []

    max_x = max(ox)
    max_y = max(oy)
    min_x = min(ox)
    min_y = min(oy)


    #sample N points 
    while len(randomx) <= 500:
        x = (np.random.random() * (max_x - min_x)) + min_x
        y = (np.random.random() * (max_y - min_y)) + min_y

        dis, _ = NN.query([x, y])

        if dis >= rr:
            randomx.append(x)
            randomy.append(y)

    #including start and goal to our randomly sampled points 
    randomx.append(sx)
    randomx.append(gx)

    randomy.append(sy)
    randomy.append(gy)

    return randomx, randomy

def changeToFinalCoords(finalpath):
    # change pixel co ords of final path to world co ords
    for i in range(finalpath.shape[0]):
        finalpath[i][0],finalpath[i][1]=changeToWorldCoords(round(finalpath[i][0]),round(finalpath[i][1]))

    return finalpath


def main():
    print(__file__ + " start.")

    
    '''
    params:
    sx: start x coord
    sy: start y coord
    gx: goal x
    gy: goal y
    
    '''

    # USED THESE FOR TESTING WITH REAL WORLD CO ORDS
    # CAN COMMENT THESE OUT AND UNCOMMENT THE INPUT() LINES ABOVE-> REPLACE SX AND SY WITH CURR BOT POS
    # GX GY CAN BE ANY CO ORD THE BOT CAN REACH
    # sx= 4.73219831575
    # sy=4.08815854895

    # Get initial state of turtlebot

    #UNCOMMMENT 
    pid = PID()
    state = pid.get_state()
    sx = state.pose.position.x
    sy = state.pose.position.y

    #get goal coordinates from terminal 
    gx=float(input('Input the goal x value:'))
    
    gy=float(input('Input the goal y value:'))
    
    # gx=3.78583484097
    # gy=6.65431327362


    #have to change to pixel co ords

    sx,sy=changeToPixelCoords(sx,sy)
    gx,gy=changeToPixelCoords(gx,gy)
    print('Start co-ords in pixel co-ords:',sx,sy)
    print('Goal co-ords in pixel co-ords',gx,gy)

    robot_radius = 5.0  # [m]

    imgArr=[]
    with open(os.path.join(sys.path[0], "mapArray.txt")) as textFile:
        for line in textFile:
            lines=line.split(',')
            imgArr.append(lines)

    imgArr=np.array(imgArr).astype(int)

    '''
    from imgArr, take in the obstacle coordinates where an obstacle is 1 
    ox: obstacle x
    oy: obstacle y
    '''

    ox = []
    oy = []
    for i in range(imgArr.shape[0]): 
        for j in range(imgArr.shape[1]):
            # 1 is an obstacle
            if(imgArr[i][j]==1):
                ox.append(i)
                oy.append(j)


    '''
    run prm which returns 
    px: path x
    py: path y
    '''

    px, py = prm(sx, sy, gx, gy, ox, oy, robot_radius)

    assert px, 'Path not found'

    finalpath=np.column_stack((px,py))
   
    # change to world co ords
    finalpath=changeToFinalCoords(finalpath)
    print(finalpath)
    return (finalpath)

#if __name__ == '__main__':
path=main()
    
