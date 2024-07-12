#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from src.turtlebot3_dqn.respawnGoal import Respawn

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.previous_distance = 0 # MODIFS !!!!

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)
        if min_range > min(scan_range) > 0:
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        if current_distance < 0.35:
            self.get_goalbox = True

        return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done
    

    def setReward(self, state, done, action):
        yaw_reward = []
        current_distance = state[-3]
        heading = state[-4]
        distance_obstacle = state[-2]
        reward = 0
        

        for i in range(5):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)  

        # recompense qui varie en fct de la distance
        if current_distance < 0.7:
            reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate * 15)  
        elif current_distance < 1.4:
            reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate * 10)  
        elif current_distance < 2.1:
            reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate * 5) 
        elif current_distance < 2.8:
            reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate * 1)  
        else :
            reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate * 20)
        
        """
        # Récompenses pour la réduction de la distance à l'objectif
        previous_distance = self.previous_distance
        distance_reward = (previous_distance - current_distance) * 50 # 700  # Récompense basée sur l'amélioration de la distance
        self.previous_distance = current_distance

        reward += distance_reward

        # Récompenses pour la direction
        direction_reward = (1 - abs(heading)) * 50 #300  # Récompense basée sur la direction
        reward += direction_reward


        
        # donne des recompenses de plus en plus grande qd il approche de la cible
        if current_distance < 0.4: 
            #rospy.loginfo("0.40m d obstacle !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            reward += 200 / current_distance
        elif current_distance < 0.8: 
            #rospy.loginfo("0.80m d obstacle !!!!!!!!!!!!!!!!!!!!!!!")
            reward += 100 / current_distance
        elif current_distance < 1.2:
            #rospy.loginfo("1.2m d obstacle !!!!!!!!!!!")
            reward += 50 / current_distance
        elif current_distance < 1.6: 
            #rospy.loginfo("1.6m d obstacle !!!!")
            reward += 25 / current_distance
        elif current_distance < 2 : 
            #rospy.loginfo("2m d obstacle !")
            reward += 10 / current_distance
        elif current_distance < 2.4: 
            #rospy.loginfo("2.4m d obstacle")
            reward += 5 / current_distance
        else :
            #rospy.loginfo("Loin de fou")
            reward += -50 / current_distance"""
        

        # MODIFS APPROXIMATIVES !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # prend cher si y se rapproche des obstacles
        """if distance_obstacle < 0.30:
            reward += -300
        elif distance_obstacle < 0.45:
            reward += -150
        elif distance_obstacle < 0.6:
            reward += -30 """
        

        if done:
            rospy.loginfo("Collision!!")
            reward += -1000 #-5000 
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward += 2000 #8000
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False
            done = True  # MODIFS !!!!

        return reward, done  # MODIFS !!!!
    

    def step(self, action): 
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data)
        reward, done = self.setReward(state, done, action) # MODIFS !!!!

        return np.asarray(state), reward, done 

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data) 

        return np.asarray(state) 