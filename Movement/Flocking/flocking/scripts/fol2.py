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
velocity = rospy.Publisher('/robot3/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

rate = rospy.Rate(10.0)
ctrl_c = False


def compute_alignment():
    #Find mean direction of neighboring agents.
    mean_velocity = Vector2()
    mean_velocity.x = (vel_leaderx + vel_robot2x)/2
    mean_velocity.y = (vel_leadery + vel_robot2y)/2
    print('alignment: ', mean_velocity)
    return mean_velocity

def compute_cohesion():
    #Find mean position of neighboring agents.
    mean_position = Vector2()
    mean_position.x = (pos_leaderx + pos_robot2x)/2
    mean_position.y = (pos_leadery + pos_robot2y)/2
    mean_position.x -= pos_robot3x
    mean_position.y -= pos_robot3y
    print('cohesion: ', mean_position)
    return mean_position

def compute_separation():
    #Find distance between neighboring agents
    vec1 = Vector2()
    vec1.x = pos_robot3x - pos_leaderx
    vec1.y = pos_robot3y - pos_leadery
    normal = vec1.norm()
    #distance = math.sqrt(pow(pos_robot3x-pos_robot3x, 2)+ pow(pos_leadery-pos_leadery, 2))
    vec1 = vec1 * normal
        
    vec2 = Vector2()
    vec2.x = pos_robot3x - pos_robot2x
    vec2.y = pos_robot3y - pos_robot2y
    normal = vec2.norm()
    #distance = math.sqrt(pow(pos_robot3x-pos_robot3x, 2) + pow(pos_robot2y-pos_robot2y,2))
    vec2 = vec2 * normal

    direction = Vector2()
    direction.x = - vec1.x - vec2.x
    direction.y = - vec1.y - vec2.y
    print('separation: ', direction)
    return direction

def compute_migration():
    #Find vector pointing to migration point
    migration_point = Vector2()
    migration_point.x = pos_leaderx - pos_robot3x
    migration_point.y = pos_leadery - pos_robot3y
    print('migration: ', migration_point)
    return migration_point

def compute_velocity():

    # Compute all the components.
    alignment = compute_alignment()
    cohesion = compute_cohesion()
    separation = compute_separation()
    migration = compute_migration()

    print("alignment: ", alignment)
    print("cohesion: ", cohesion)
    print("separation: ", separation)
    print("migration: ", migration)

    alignment_factor = 1.5
    cohesion_factor = 1.0
    separation_factor = 0.0
    migration_factor = 1.0

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
    print "shutdown time! Stop the robot"
    ctrl_c = True

rospy.on_shutdown(shutdownhook)

while not ctrl_c:
    v = Vector2()
    v = compute_velocity()
    #cmd = geometry_msgs.msg.Twist()
    #cmd.linear.x = v.x
    #cmd.angular.z = v.y   
    #velocity.publish(cmd)
    #print("robot2 results")
    #c = compute_cohesion()
    #a = compute_alignment()
    #m = compute_migration()
    #s = compute_separation()
    rate.sleep()

