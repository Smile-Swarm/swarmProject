#!/usr/bin/env python
import rospy
import math
import tf
import random
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from math import atan2
import numpy 

pos_x = 0.0
pos_y = 0.0

vel_x = 0.0
vel_y = 0.0
vel_w = 0.0

phi = 0.0

grid_x = 0.0
grid_y = 0.0

Length = 20
Width = 20

delta_T = 6 # Simulation unit time step (second)
NoofTimeInterval = 3 #Total number of simulation time steps
V = 50 # Speed (meter/second)
j = 0 # Time counter to record the elapsed duration of maintaining the current turning radius
R = 3 # Initialization of the turning radius (meter)

def robot_info(msg):
    global pos_x, pos_y, pos_z, vel_x, vel_y, vel_w, phi
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y

    grid_x = pos_x + 10
    grid_y = pos_y + 10

    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    vel_w = msg.twist.twist.angular.z

    rot_q = msg.pose.pose.orientation
    (roll, pitch, phi) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node('ground_point')
position = rospy.Subscriber('/robot1/odom', Odometry, robot_info)
move = rospy.Publisher('/robot1/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

rate = rospy.Rate(10.0)
ctrl_c = False

# Initialization of the turn center
center_x = pos_x + (R * math.sin(phi))
center_y = pos_y - (R * math.cos(phi))

W = V / R # Initialization of angular velocity (radian/s)
theta = W * delta_T # Initialization of the turn angle (radian)
#varian = 3.105e-5 # Variance of the Gaussian variable, determining the preference between straight trajectories and turns
ExponentialMean = 100 # Mean duration between the changes of turning centers (second)
changetime = numpy.random.exponential(ExponentialMean) / delta_T

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
a = 5
b = 5
while not ctrl_c:
    speed = geometry_msgs.msg.Twist()
    count += 1
    if count == NoofTimeInterval:
        count = 0
        j += 1
    #When the waiting time is reached, a new waiting time and a new turn radius are generated. Turn angle and turn radius are calculated accordingly.
        if j > changetime:
            changetime = numpy.random.exponential(ExponentialMean) / delta_T
            j = 0 
            #R_d = numpy.random.normal(0, varian)
            #R = 1 / R_d
            R = numpy.random.uniform(-5,5)
            W = V / R
            #print(R_d, R, W)
            theta = W * delta_T
            center_x = grid_x + (R * math.sin(phi))
            center_y = grid_y - (R * math.cos(phi))
            
            #Update the X location, Y location and the heading angle
            x = center_x + (R * math.sin(theta-phi))
            y = center_y + (R * math.cos(theta-phi))
            #phi = phi-theta

            #Reflection Boundary model
            a = 2*Length*abs((x/2/Length-math.floor(x/2/Length+0.5)))
            b = 2*Width*abs((y/2/Width-math.floor(y/2/Width+0.5)))
            a -= 10
            a -= 10

            if a < -10:
                #a = numpy.random.uniform(-9, -3)
                a = -7
            elif a > 10:
                #a = numpy.random.uniform(3, 9)
                a = 7
            if b < -10:
                #b = numpy.random.uniform(-9, -3)
                b = -7
            elif b > 10:
                #b = numpy.random.uniform(3, 9)
                b = 7

            #print("a, b")
            #print(a, b)

    inc_x = a - pos_x
    inc_y = b - pos_y
    angle = atan2(inc_y, inc_x)
    
    #print(angle - phi)
    if (abs(inc_x) <= 0.3) and (abs(inc_y) <= 0.3):
        speed.linear.x = 0.0
        speed.angular.z = 0.0
    else:
        #rotate counter-clockwise
        if angle - phi > 0.3:
            speed.linear.x = 0.3
            speed.angular.z = 0.6
        #rotate counter-clockwise
        elif angle - phi < -0.3:
            speed.linear.x = 0.3
            speed.angular.z = -0.6
        #move forward
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
    move.publish(speed)

