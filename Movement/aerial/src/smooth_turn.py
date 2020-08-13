#!/usr/bin/env python
import rospy
import math
import tf
import random
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from math import atan2

pos_x = 0.0
pos_y = 0.0
pos_z = 0.0
vel_x = 0.0
vel_y = 0.0
vel_w = 0.0
theta = 0.0


def robot_info(msg):
    global pos_x, pos_y, pos_z, vel_x, vel_y, vel_w, theta
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    pos_z = msg.pose.pose.position.z

    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    vel_w = msg.twist.twist.angular.z

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node('aerial')
position = rospy.Subscriber('ground_truth/state', Odometry, robot_info)
move = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

rate = rospy.Rate(10.0)
ctrl_c = False

def shutdownhook():
    global ctrl_c
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0  
    move.publish(cmd)
    print "shutdown time! Stop the robot"
    ctrl_c = True

rospy.on_shutdown(shutdownhook)

count = 0
rand_x = 0
rand_y = 0
while not ctrl_c:
    speed = geometry_msgs.msg.Twist()
    count += 1
    if count == 50000:
        count = 0
        print("Robot Heading:")
        #print(pos_x)
        #print(pos_y)
        #print(vel_x)
        #print(vel_y)
        #print(vel_w)
        #theta_perp = -1/theta
        #print(theta)
        #print(theta_perp)
        #b = pos_y - (theta_perp * pos_x)
        rand_x = random.uniform(pos_x - 5, pos_x + 5)
        if rand_x >= 10:
            rand_x = 7
        if rand_x <= -10:
            rand_x = -7
        rand_y = random.uniform(pos_y - 5, pos_y + 5)
        if rand_y >= 10:
            rand_y = 7
        if rand_y <= -10:
            rand_y = -7
        print(rand_x, rand_y)

    inc = 2 - pos_z 
    if abs(inc) <= 0.7:
        speed.linear.z = 0.0
    else:
        speed.linear.z = 0.2

    inc_x = rand_x - pos_x
    inc_y = rand_y - pos_y
    angle = atan2(inc_y, inc_x)
        
    if (abs(inc_x) <= 0.1) and (abs(inc_y) <= 0.1):
        speed.linear.x = 0.0
        speed.angular.z = 0.0
    else:
        if angle - theta > 0.2:
            speed.linear.x = 0.3
            speed.angular.z = 0.2
        #rotate counter-clockwise
        elif angle - theta < -0.2:
            speed.linear.x = 0.3
            speed.angular.z = -0.2
        #move forward
        else:
            speed.linear.x = 0.3
            speed.angular.z = 0.0

    #cmd.linear.x = 0.2
    #cmd.angular.z = 0.0
    move.publish(speed)

