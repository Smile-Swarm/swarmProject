#!/usr/bin/env python
import sys
import rospy
import math
import tf
import numpy as np
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point
from tf.transformations import euler_from_quaternion
import util
from util import Vector2
from math import atan2

leader = [0, 0, 0, 0]
robot2 = [0, 0, 0, 0]
robot3 = [0, 0, 0, 0]
robot4 = [0, 0, 0, 0]
all_agents = [0, 0]
nearest_agents = []

pos_leaderx = 0.0
pos_leadery = 0.0
vel_leaderx = 0.0 
vel_leadery = 0.0

pos_robot2x = 0.0
pos_robot2y = 0.0
vel_robot2x = 0.0 
vel_robot2y = 0.0

pos_robot3x = 0.0
pos_robot3y = 0.0
vel_robot3x = 0.0 
vel_robot3y = 0.0
theta = 0

#pos_robot4x = 0.0
#pos_robot4y = 0.0
#vel_robot4x = 0.0 
#vel_robot4y = 0.0

def leader_info(msg):
    global pos_leaderx, pos_leadery, vel_leaderx, vel_leadery, leader
    pos_leaderx = msg.pose.pose.position.x 
    pos_leadery = msg.pose.pose.position.y 
    
    vel_leaderx = msg.twist.twist.linear.x
    vel_leadery = msg.twist.twist.angular.z

def robot2_info(msg):
    global pos_robot2x, pos_robot2y, vel_robot2x, vel_robot2y, robot2
    pos_robot2x = msg.pose.pose.position.x
    pos_robot2y = msg.pose.pose.position.y

    vel_robot2x = msg.twist.twist.linear.x
    vel_robot2y = msg.twist.twist.angular.z

def robot3_info(msg):
    global pos_robot3x, pos_robot3y, vel_robot3x, vel_robot3y, theta, robot3
    pos_robot3x = msg.pose.pose.position.x
    pos_robot3y = msg.pose.pose.position.y
    
    vel_robot3x = msg.twist.twist.linear.x
    vel_robot3y = msg.twist.twist.angular.z 

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

#def robot4_info(msg):
 #   global pos_robot4x, pos_robot4y, vel_robot4x, vel_robot4y, theta, robot4
  #  pos_robot4x = msg.pose.pose.position.x
   # pos_robot4y = msg.pose.pose.position.y
    
   # vel_robot4x = msg.twist.twist.linear.x
    #vel_robot4y = msg.twist.twist.angular.z 

rospy.init_node('follower2')
leader_pos = rospy.Subscriber('/leader_pose', Odometry, leader_info)
robot2_pos = rospy.Subscriber('/robot2/odom', Odometry, robot2_info)
robot3_pos = rospy.Subscriber('/robot3/odom', Odometry, robot3_info)
#robot4_pos = rospy.Subscriber('/robot4/odom', Odometry, robot3_info)
velocity = rospy.Publisher('/robot3/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

rate = rospy.Rate(10.0)
ctrl_c = False

def compute_nearest_agents():
    nearest = []
    for agent in all_agents:
        distance = np.sqrt((leader[0] - agent[0]) ** 2 + (leader[1] - agent[1]) ** 2)
        if distance < 8:
            nearest.append(agent)
    return nearest

def compute_alignment(nearest_agents):
    #Find mean direction of neighboring agents.
    mean_velocity = Vector2()
    for agent in nearest_agents:
        mean_velocity.x += agent[2]
        mean_velocity.y += agent[3]
    mean_velocity.x = mean_velocity.x / len(nearest_agents)
    mean_velocity.y = mean_velocity.y / len(nearest_agents)
    return mean_velocity

def compute_cohesion(nearest_agents):
    #Find mean position of neighboring agents.
    mean_position = Vector2()
    for agent in nearest_agents:
        mean_position.x = agent[0] 
        mean_position.y = agent[1]
    mean_position.x = mean_position.x / len(nearest_agents)
    mean_position.y = mean_position.x / len(nearest_agents)
    mean_position.x -= robot3[0]
    mean_position.y -= robot3[1]
    #print("cohesion: ", mean_position)
    return mean_position

def compute_separation(nearest_agents):
    #Find distance between neighboring agents
    vec1 = Vector2()
    direction = Vector2()
    for agent in nearest_agents:
        vec1.x = robot3[0] - agent[0]
        vec1.y = robot3[1] - agent[1]
        distance = vec1.norm() #magnitude of vector = length
        vec1 = vec1 / distance #convert vec1 to unit vector
        vec1 = vec1 / distance #magnitude in opposite direction is inversely proportional to distance btw robots  
        direction += vec1
    #print("separation: ", direction)
    return direction

def compute_migration():
    #Find vector pointing to migration point
    migration_point = Vector2()
    migration_point.x = leader[0] - robot3[0]
    migration_point.y = leader[1] - robot3[1]
    #print("migration*:   %s", migration_point)
    return migration_point

def compute_velocity():

    leader[0] = pos_leaderx
    leader[1] = pos_leadery
    leader[2] = vel_leaderx
    leader[3] = vel_leadery

    robot2[0] = pos_robot2x
    robot2[1] = pos_robot2y
    robot2[2] = vel_robot2x
    robot2[3] = vel_robot2y

    robot3[0] = pos_robot3x
    robot3[1] = pos_robot3y
    robot3[2] = vel_robot3x
    robot3[3] = vel_robot3y 

    #robot4[0] = pos_robot4x
    #robot4[1] = pos_robot4y
    #robot4[2] = vel_robot4x
    #robot4[3] = vel_robot4y 

    all_agents[0] = leader
    all_agents[1] = robot2
    #all_agents[2] = robot4

    nearest_agents = compute_nearest_agents()

    # Compute all the components.
    alignment = compute_alignment(nearest_agents)
    cohesion = compute_cohesion(nearest_agents)
    separation = compute_separation(nearest_agents)
    migration = compute_migration()

    #print('alignment: ', alignment)
    #print('cohesion: ', cohesion)
    #print('separation: ', separation)
    #print('migration: ', migration)

    alignment_factor = 0.2
    cohesion_factor = 0.2
    separation_factor = 0.5
    migration_factor = 1

    # Add components together and limit the output.
    velocity = Vector2()
    velocity += alignment * alignment_factor
    velocity += cohesion * cohesion_factor
    velocity += separation * separation_factor
    velocity += migration * migration_factor

    #print('velocity: ', velocity)

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
    #print("robot3 results")
    speed = geometry_msgs.msg.Twist()

    v = Vector2()
    v = compute_velocity()

    inc_x = v.linear.x
    inc_y = v.linear.y

    goal = Point()
    goal.x = robot3[0] + inc_x
    goal.y = robot3[1] + inc_y

    #print("2")
    #print(goal.x, goal.y)

    dist_goal = np.sqrt((goal.x - robot3[0]) ** 2 + (goal.y - robot3[1]) ** 2)

    angle_to_goal = atan2(inc_y, inc_x)
    print(angle_to_goal)
    print(theta)
        
    #if no error (with margin) then no movement
    if (abs(inc_x) <= 0.4) and (abs(inc_y) <= 0.4):
        speed.linear.x = 0.0
        speed.angular.z = 0.0   
    #if there is an error different actions must be taken
    else:
        #rotate clockwise
        if angle_to_goal - theta > 0.3:
            speed.linear.x = 0.1
            speed.angular.z = 0.3
        #rotate counter-clockwise
        elif angle_to_goal - theta < -0.3:
            speed.linear.x = 0.1
            speed.angular.z = -0.3
        #move forward
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0

    velocity.publish(speed)
    rate.sleep() 

