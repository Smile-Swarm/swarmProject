#!/usr/bin/env python

import rospy, ast
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
 
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
    
#set goal (Set Point)
goal.x = 5
goal.y = 5
print('Moving to point: (%d, %d)' % (goal.x, goal.y))

#START OF PID CONTROLL
while not rospy.is_shutdown():
    #find the difference between the SP and PV (error)
    inc_x = goal.x -x
    inc_y = goal.y -y
    angle_to_goal = atan2(inc_y, inc_x)
        
    #if no error (with margin) then no movement
    if (abs(inc_x) <= 0.1) and (abs(inc_y) <= 0.1):
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        break
        
    #if there is an error different actions must be taken
    else:
        #rotate clockwise
        if angle_to_goal - theta > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.4
        #rotate counter-clockwise
        elif angle_to_goal - theta < -0.1:
            speed.linear.x = 0.0
            speed.angular.z = -0.4
        #move forward
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
 
    pub.publish(speed)
    r.sleep()

print('Goal Reached!')
