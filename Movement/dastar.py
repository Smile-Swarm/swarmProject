#!/usr/bin/env python

import rospy, ast
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import *
import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
 
##############################################################################
# plot grid (map)
##############################################################################

grid = np.array([

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

x = 0.0
y = 0.0 
theta = 0.0
 
def newOdom(msg):
    global x
    global y
    global theta

    #get current x,y coordinates (Process Variable)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
 
rospy.init_node("speed_controller")
sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
 
speed = Twist()
r = rospy.Rate(4)
goal = Point()

#####################################################################################
# grid coordinates to matrix form
#####################################################################################

def tomatrix(a, b):
    a = a + 10
    if b > 0:
        b = 10 - b
    else:
        b = b*(-1) + 10
    return a, b

#####################################################################################
# extract coordinates from string
#####################################################################################

def getcoor(coordinate):
    string = str(coordinate)
    b,a = ast.literal_eval(string)
    b = (b-10)*(-1)
    a = a-10
    return a, b

##############################################################################
# Find current position to use for A*
##############################################################################

temp = 0
while not rospy.is_shutdown():
    current_x = x
    current_y = y
    r.sleep()
    if temp == 1:
        break
    temp = temp + 1

print("Current position:")
print(int(round(current_x)), (int(round(current_y))))

tempx = current_x
tempy = current_y

goala = input("Enter desired x coordinate:")
goalb = input("Enter desired y coordinate:")

current_x, current_y = tomatrix(x, y)
goala, goalb = tomatrix(goala, goalb)

start = (int(round(current_y)), int(round(current_x)))
goal = (goalb, goala)

#####################################################################################
# heuristic function for path scoring (actual cost between 2 nodes without obstacles)
#####################################################################################

def heuristic(a, b):
 
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

##############################################################################
# path finding function (algorithm to find shortest path)
##############################################################################

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
 
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
 
    return False

#####################################################################################
# Print path function
#####################################################################################

def printpath(route):
    print('Waypoints of shortest path: ')
    for point in range(0, len(route)):
        a, b = getcoor(route[point])
        print(a,b)

route = astar(grid, start, goal) #call astar function but route is end to start
route = route[::-1] #reverse the order
printpath(route)

##############################################################################
# PID
##############################################################################

obstacle = (4,-2)
point = 0 #counter
while point < len(route):
    #get coordinate
    a, b = getcoor(route[point])
    if (obstacle == (a,b)):
        print("collision detected")
        current_x, current_y = tomatrix(x, y)
        grid [12, 14] = 1
        start = (int(round(current_y)), int(round(current_x)))
        route = astar(grid, start, goal) #call astar function but route is end to start
        route = route[::-1] #reverse the order
        printpath(route)
        point = 0
        continue
    else:
        #get next coordinate
        if point + 1 < len(route):
            nexta, nextb = getcoor(route[point+1])
            if (obstacle == (nexta,nextb)):
                print("collision detected")
                current_x, current_y = tomatrix(x, y)
                grid [12, 14] = 1
                start = (int(round(current_y)), int(round(current_x)))
                route = astar(grid, start, goal) #call astar function but route is end to start
                route = route[::-1] #reverse the order
                printpath(route)
                point = 0
                continue
            if point == 0:
                angle = atan2(b-int(round(tempy)),a-int(round(tempx))) #angle
                nextangle = atan2(nextb-b,nexta-a) #nextangle
            else:
                lasta, lastb = getcoor(route[point-1])
                angle = atan2(b-lastb,a-lasta) #angle
                nextangle = atan2(nextb-b,nexta-a) #nextangle

            temp = point+1
            if angle == nextangle:
                while angle == nextangle:
                    if temp + 1 < len(route):
                        nnexta, nnextb = getcoor(route[temp+1])
                        if (obstacle == (nnexta,nnextb)):
                            print("collision detected")
                            current_x, current_y = tomatrix(x, y)
                            grid [12, 14] = 1
                            start = (int(round(current_y)), int(round(current_x)))
                            route = astar(grid, start, goal) #call astar function but route is end to start
                            route = route[::-1] #reverse the order
                            printpath(route)
                            point = 0
                            continue
                
                        nextangle = atan2(nnextb-b,nnexta-a) #angle
                        goalx = nexta
                        goaly = nextb
                        nexta = nnexta
                        nextb = nnextb
                        temp = temp + 1
                    else:
                        goalx = nexta
                        goaly = nextb
                        angle = 0
                        nextangle = 1
                point = temp
            else:
                #set goal (Set Point)
                goalx = a
                goaly = b
                point = point + 1
        else:
            goalx = a
            goaly = b
            point = point +1
    
        print('Moving to point: (%d, %d)' % (goalx, goaly))

        v = 0# default linear velocity
        #START OF PID CONTROL
        while not rospy.is_shutdown():
            wgain = 10.0 # Gain for the angular velocity [rad/s / rad]
            vgain = 0.2
            vmax = 0.8 # Linear velocity when far away [m/s]
            vmin = 0.4
            distThresh = 0.1 # Distance treshold [m]
            w = 0 # default angluar velocity
            distance = sqrt((x-goalx)**2+(y-goaly)**2) #distance btw current & goal
            if (distance > distThresh):
            
                #angular velocity control
                desireYaw = atan2(goaly-y,goalx-x) #yaw = angle
                u = desireYaw-theta
                if (u > 0.1 or u < -0.1):
                    bound = atan2(sin(u),cos(u)) 
                    w = min(0.7 , max(-0.7, wgain*bound)) # w = yaw rate = angular velocity
                    v = 0.25 #small amount of linear velocity while turning
            
                #linear velocity control
                else:
                    #speed up until constant
                    if distance > 0.8:
                        if v < vmax:
                            v = v + v*vgain
                            if v > vmax:
                                v = vmax
                        else:
                            v = vmax

                    #start slowing down
                    else:
                        if v > vmin:
                            v = v - v*vgain
                            if v < vmin:
                                v = vmin
                        else:
                            v = vmin
            else:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                break
        
            # Publish
            speed.linear.x = v
            speed.angular.z = w
 
            pub.publish(speed)

print('Goal Reached!')
