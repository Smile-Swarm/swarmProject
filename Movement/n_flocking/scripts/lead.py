#!/usr/bin/env python
import rospy
import tf
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

px1 = 0.0
py1 = 0.0
pz1= 0.0

vx1 = 0.0 
vz1 = 0.0

def callback(msg):
    global px1, py1, pz1, ox1, oy1, oz1, ow1, vx1, vz1
    px1 = msg.pose.pose.position.x
    py1 = msg.pose.pose.position.y
    pz1 = msg.pose.pose.position.z

    vx1 = msg.twist.twist.linear.x
    vz1 = msg.twist.twist.angular.z


rospy.init_node('leader')
sub = rospy.Subscriber('/robot1/odom', Odometry, callback)
pub = rospy.Publisher('/leader_pose', Odometry, queue_size = 10)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    pose_msg = Odometry()
    pose_msg.pose.pose.position.x = px1
    pose_msg.pose.pose.position.y = py1
    pose_msg.pose.pose.position.z = pz1

    pose_msg.twist.twist.linear.x = vx1
    pose_msg.twist.twist.angular.z = vz1

    pub.publish(pose_msg)
    rate.sleep()



