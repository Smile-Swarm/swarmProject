#!/usr/bin/env python
# -*- coding: utf-8 -*-
# from __future__ import division

import math
import rospy
from geometry_msgs.msg import Twist
from util import Vector2, angle_diff, MAFilter


def get_agent_velocity(agent):
    vel = Vector2()
    vel.x = agent.twist.twist.linear.x
    vel.y = agent.twist.twist.linear.y
    return vel

def get_agent_position(agent):
    pos = Vector2()
    pos.x = agent.pose.pose.position.x
    pos.y = agent.pose.pose.position.y
    return pos

class Boid(object):

    def __init__(self, initial_velocity_x, initial_velocity_y, wait_count, start_count, frequency):
        self.position = Vector2()
        self.velocity = Vector2()
        self.mass = 1.0               # Mass of robot in kilograms
        self.wait_count = wait_count    # Waiting time before starting
        self.start_count = start_count  # Time during which initial velocity is being sent
        self.frequency = frequency      # Control loop frequency

        # Set initial velocity
        self.initial_velocity = Twist()
        self.initial_velocity.linear.x = 0.0
        self.initial_velocity.linear.y = 0.0

    def update_parameters(self, params):
        self.alignment_factor = params['alignment_factor']
        self.cohesion_factor = params['cohesion_factor']
        self.separation_factor = params['separation_factor']
        self.avoid_factor = params['avoid_factor']
        self.max_speed = params['max_speed']
        self.max_force = params['max_force']
        self.friction = params['friction']
        self.crowd_radius = params['crowd_radius']
        self.search_radius = params['search_radius']
        self.avoid_radius = params['avoid_radius']

        # Scaling is calculated so that force is maximal when agent is
        # 0.85 * search_radius away from obstacle.
        self.avoid_scaling = 1 / ((0.85 * self.search_radius) ** 2 * self.max_force)

        # Scaling is calculated so that cohesion and separation forces
        # are equal when agents are crowd_radius apart.
        self.separation_scaling = self.search_radius / self.crowd_radius ** 3 / self.max_force

    def compute_alignment(self, nearest_agents):
        mean_velocity = Vector2()
        steer = Vector2()
        # Find mean direction of neighboring agents.
        for agent in nearest_agents:
            agent_velocity = get_agent_velocity(agent)
            mean_velocity += agent_velocity
        print("alignment*:   %s", mean_velocity)

        # Steer toward calculated mean direction with maximum velocity.
        if nearest_agents:
            mean_velocity.set_mag(self.max_speed)
            steer = mean_velocity - self.velocity
            steer.limit(self.max_force)
        return steer

    def compute_cohesion(self, nearest_agents):
        mean_position = Vector2()
        direction = Vector2()
        # Find mean position of neighboring agents.
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            mean_position += agent_position

        # Apply force in the direction of calculated mean position.
        # Force is proportional to agents' distance from the mean.
        if nearest_agents:
            direction = mean_position / len(nearest_agents)
            print("cohesion*:    %s", direction)
            d = direction.norm()
            direction.set_mag((self.max_force * (d / self.search_radius)))
        return direction

    def compute_separation(self, nearest_agents):
        direction = Vector2()
        count = 0

        # Calculate repulsive force for each neighboring agent in sight.
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            d = agent_position.norm()
            if d < self.crowd_radius:
                count += 1
                agent_position *= -1        # Make vector point away from other agent.
                agent_position.normalize()  # Normalize to get only direction.
                # Vector's magnitude is proportional to inverse square of the distance between agents.
                agent_position = agent_position / (self.separation_scaling * d**2)
                direction += agent_position

        if count:
            # Devide by number of close-by agents to get average force.
            direction /= count
            direction.limit(2 * self.max_force)  # 2 * max_force gives this rule a slight priority.
        print("separation*:  %s", direction)
        return direction

    def compute_velocity(self, my_agent, nearest_agents, avoids):

        # While waiting to start, send zero velocity and decrease counter.
        if self.wait_count > 0:
            self.wait_count -= 1
            print("wait " + '{}'.format(self.wait_count))
            print("velocity:\n%s", Twist().linear)
            return Twist(), None

        # Send initial velocity and decrease counter.
        elif self.start_count > 0:
            self.start_count -= 1
            print("start " + '{}'.format(self.start_count))
            print("velocity:\n%s", self.initial_velocity.linear)
            return self.initial_velocity, None

        # Normal operation, velocity is determined using Reynolds' rules.
        else:
            self.velocity = get_agent_velocity(my_agent)
            self.old_heading = self.velocity.arg()
            self.old_velocity = Vector2(self.velocity.x, self.velocity.y)
            print("old_velocity: %s", self.velocity)

            # Compute all the components.
            alignment = self.compute_alignment(nearest_agents)
            cohesion = self.compute_cohesion(nearest_agents)
            separation = self.compute_separation(nearest_agents)
            avoid = self.compute_avoids(avoids)

            print("alignment:    %s", alignment)
            print("cohesion:     %s", cohesion)
            print("separation:   %s", separation)
            print("avoid:        %s", avoid)

            # Add components together and limit the output.
            force = Vector2()
            force += alignment * self.alignment_factor
            force += cohesion * self.cohesion_factor
            force += separation * self.separation_factor
            force.limit(self.max_force)

            # If agent is moving, apply constant friction force.
            # If agent's velocity is less then friction / 2, it would get
            # negative velocity. In this case, just stop it.
            if self.velocity.norm() > self.friction / 2:
                force += self.friction * -1 * self.velocity.normalize(ret=True)
            else:
                self.velocity = Vector2()

            acceleration = force / self.mass

            # Calculate total velocity (delta_velocity = acceleration * delta_time).
            self.velocity += acceleration / self.frequency
            self.velocity.limit(self.max_speed)

            print("force:        %s", force)
            print("acceleration: %s", acceleration / self.frequency)
            print("velocity:     %s\n", self.velocity)

            # Return the the velocity as Twist message.
            vel = Twist()
            vel.linear.x = self.velocity.x
            vel.linear.y = self.velocity.y

            return vel
