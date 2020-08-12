#!/usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point
import util
from util import Vector2

#pos_leader = (0.0, 0.0)
#vel_leader = (0.0, 0.0)
pos_leaderx = 0.0
pos_leadery = 0.0
vel_leaderx = 0.0 
vel_leadery = 0.0

#pos_robot2 = (0.0, 0.0)
#vel_robot2 = (0.0, 0.0)
pos_robot2x = 0.0
pos_robot2y = 0.0
vel_robot2x = 0.0 
vel_robot2y = 0.0

#pos_robot3 = (0.0, 0.0)
#vel_robot3 = (0.0, 0.0)
pos_robot3x = 0.0
pos_robot3y = 0.0
vel_robot3x = 0.0 
vel_robot3y = 0.0

def leader_info(msg):
    #pos_leader = Vector2()
    global pos_leaderx, pos_leadery, vel_leaderx, vel_leadery
    pos_leaderx = msg.pose.pose.position.x 
    pos_leadery = msg.pose.pose.position.y 
    
    #vel_leader = Vector2()
    vel_leaderx = msg.twist.twist.linear.x
    vel_leadery = msg.twist.twist.angular.z

def robot2_info(msg):
    global pos_robot2x, pos_robot2y, vel_robot2x, vel_robot2y
    #pos_robot2 = Vector2()
    pos_robot2x = msg.pose.pose.position.x
    pos_robot2y = msg.pose.pose.position.y

    or_x = msg.pose.pose.orientation.x
    or_y = msg.pose.pose.orientation.y
    or_z = msg.pose.pose.orientation.z
    or_w = msg.pose.pose.orientation.w

    #vel_robot2 = Vector2()
    vel_robot2x = msg.twist.twist.linear.x
    vel_robot2y = msg.twist.twist.angular.z

def robot3_info(msg):
    global pos_robot3x, pos_robot3y, vel_robot3x, vel_robot3y
    #pos_robot3 = Vector2()
    pos_robot3x = msg.pose.pose.position.x
    pos_robot3y = msg.pose.pose.position.y
    
    #vel_robot3 = Vector2()
    vel_robot3x = msg.twist.twist.linear.x
    vel_robot3y = msg.twist.twist.angular.z

rospy.init_node('follower')
leader_pos = rospy.Subscriber('/leader_pose', Odometry, leader_info)
robot2_pos = rospy.Subscriber('/robot2/odom', Odometry, robot2_info)
robot3_pos = rospy.Subscriber('/robot3/odom', Odometry, robot3_info)
velocity = rospy.Publisher('/robot2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

rate = rospy.Rate(10.0)
ctrl_c = False

def compute_alignment():
    #Find mean direction of neighboring agents.
    mean_velocity = Vector2()
    mean_velocity.x = (vel_leaderx + vel_robot3x)/2
    mean_velocity.y = (vel_leadery + vel_robot3y)/2
    print("alignment: ", mean_velocity)
    return mean_velocity

def compute_cohesion():
    #Find mean position of neighboring agents.
    mean_position = Vector2()
    mean_position.x = (pos_leaderx + pos_robot3x)/2 
    mean_position.y = (pos_leadery + pos_robot3y)/2
    mean_position.x -= pos_robot2x
    mean_position.y -= pos_robot2y
    print("cohesion: ", mean_position)
    return mean_position

def compute_separation():
    #Find distance between neighboring agents
    vec1 = Vector2()
    vec1.x = pos_robot2x - pos_leaderx
    vec1.y = pos_robot2y - pos_leadery
    distance = vec1.norm() #magnitude of vector = length
    vec1 = vec1 / distance #convert vec1 to unit vector
    vec1 = vec1 / distance #magnitude in opposite direction is inversely proportional to distance btw robots  
        
    vec2 = Vector2()
    vec2.x = pos_robot2x - pos_robot3x
    vec2.y = pos_robot2y - pos_robot3y
    distance = vec2.norm() #magnitude of vector = length
    vec2 = vec2 / distance #convert vec1 to unit vector
    vec2 = vec2 / distance #magnitude in opposite direction is inversely proportional to distance btw robots 

    direction = Vector2()
    direction = vec1 + vec2
    print("separation: ", direction)
    return direction

def compute_migration():
    #Find vector pointing to migration point
    migration_point = Vector2()
    migration_point.x = pos_leaderx - pos_robot2x
    migration_point.y = pos_leadery - pos_robot2y
    print("migration*:   %s", migration_point)
    return migration_point

def compute_velocity():

    # Compute all the components.
    alignment = compute_alignment()
    cohesion = compute_cohesion()
    separation = compute_separation()
    migration = compute_migration()

    print('alignment: ', alignment)
    print('cohesion: ', cohesion)
    print('separation: ', separation)
    print('migration: ', migration)

    alignment_factor = 0.7
    cohesion_factor = 0.5
    separation_factor = 0.5
    migration_factor = 0.5

    # Add components together and limit the output.
    velocity = Vector2()
    velocity += alignment * alignment_factor
    velocity += cohesion * cohesion_factor
    velocity += separation * separation_factor
    velocity += migration * migration_factor

    print('velocity: ', velocity)

    # Return the the velocity as Twist message.
    vel = Twist()
    vel.linear.x = velocity.x
    vel.linear.y = velocity.y

    return vel

#if in shutdown, completely stop movement
def shutdownhook():
    global ctrl_c
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0  
    velocity.publish(cmd)
    print "shutdown time! Stop the robot"

    ctrl_c = True

rospy.on_shutdown(shutdownhook)

while not ctrl_c:
    print("robot1 results")
    v = Vector2()
    v = compute_velocity()

    dist_goal  = sqrt(pow(v.linear.x, 2)+pow(v.linear.x, 2)) #distance btw current & goal
    if (dist_goal  > 0.8):
        desireYaw = atan2(v.linear.y, v.linear.x) #yaw = angle
        u = desireYaw-theta2
        if (u > 0.1 or u < -0.1):
            bound = atan2(sin(u),cos(u)) 
            w = min(0.7 , max(-0.7, 10.0*bound)) # w = yaw rate = angular velocity
            v = 0.25
            #v = 0.1 #small amount of linear velocity while turning
        #linear velocity control
        else:
            #if v == 0.25:
		        #v = 1.2 
            if dist_goal > 1:
                v = v + 0.2
                if v > 2:
                    v = 2
            else:
                v = v - 0.1
                if v < 0.5:
                    v = 0.5
    else:
        v = 0.0
        w = 0.0

    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = v.linear.x
    cmd.angular.z = v.linear.y   
    velocity.publish(cmd)
    rate.sleep() 
