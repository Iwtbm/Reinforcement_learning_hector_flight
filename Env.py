#!/usr/bin/env python

import time
import numpy as np
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist, Point, Pose
from rl_control import State_feedback
from collections import defaultdict
import rospy
import tf
import math
import random

class Env(object):
    def __init__(self):
        '''
        [forward,left,backward,right,up,down] = [0,1,2] 
        
        '''

        self.threshold = 0.22 # check if collision
        self.simulation  = State_feedback()
        self.state  = []
        self.state_space = None
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        self.current_info = defaultdict(list)
        self.reward = 0.0
        self.max_height = 1
        self.setstart()
        self.goal = [1, 0]
        self.goal_range = 0.6
           
    def setstart(self):
        self.startset = [[-1, 1.1, 0.2], [2, 1, 0.2], [-1, -1, 0.2], [-1, -2.5, 0.2], [2.5, -2, 0.2]]


    def step(self, vel, action):
        '''
        Input: 1 x 6 List
        Output:Current_state,current_reward,current_status,{}
        publish the action to gazebo model and return it in several second
        '''
 
        self.simulation.give_vel(vel,Time = 0.2)
        Info =  self.Get_current_info()
        self.dist = round(math.hypot(Info['Current_Pose'][0] - self.goal[0], Info['Current_Pose'][1] - self.goal[1]), 2)
        
        self.theta_pos = tf.transformations.euler_from_quaternion(Info['position'][3:])
        done =self.Done_check()
        self.heading = self.theta_pos[2] - math.atan2(self.goal[1] - Info['Current_Pose'][1], self.goal[0] - Info['Current_Pose'][0])

        if self.heading > math.pi:
            self.heading -= 2 * math.pi
        elif self.heading < -math.pi:
            self.heading += 2 * math.pi
         
        lidar = Info['lidar'][::40]     # pick 28 lidar data
        self.state = list(lidar) + [self.dist, Info['obs_min'], Info['obs_max'], -self.heading]  ## state length: 32
        
        reward = self.setReward(self.state,done,action)
        status = done
 
        return np.asarray(self.state), reward, status
    
    def Get_current_info(self):
        '''
        Get Current information, which contains: Position,Velocity, Lidar Data
        '''        
        self.current_info['Current_Pose'] = self.simulation.pos
        self.current_info['Current_velocity'] = self.simulation.vel
        self.current_info['lidar'] = self.simulation.laser_ranges
        self.current_info['obs_max'] = self.simulation.obs_max
        self.current_info['obs_min'] = self.simulation.obs_min
        self.current_info['position'] = self.simulation.pos
        
        return self.current_info

    def Done_check(self):
        '''
        Check if the quadrotor make collision or achieve target.
        '''

        Min_dist = self.current_info['obs_min']
        
        if abs(self.theta_pos[0]) > 0.25 * math.pi or abs(self.theta_pos[1]) > 0.25 * math.pi:
            return True
            
        elif Min_dist <= self.threshold:
            
            return True

        elif self.dist < self.goal_range :
            
            return True
            
        else:
            return False

        
    def reset(self):
        '''
        reset env to random orginal position 
        
        '''

        self.reward = 0
        Start = time.clock()
        
        self.setstart()
        
        k = random.randint(0, 4)
        ort = np.random.rand() * math.pi * 2
        ort = tf.transformations.quaternion_from_euler(0, 0, ort)

        while True:
            if k == 0:
                self.simulation.set_position(self.startset[k]+[0,0,0,0])
            elif k == 3:
                ang = random.randint(0, 1)
                ort = tf.transformations.quaternion_from_euler(0, 0, math.pi * 0.5 * ang)
                self.simulation.set_position(self.startset[k]+list(ort))
            else:
                self.simulation.set_position(self.startset[k]+list(ort))

            if time.clock() - Start >=0.3:
                break
       
        t = 0

        print('Reseting...')
        while True and not rospy.is_shutdown():
            if self.simulation.pos[2] < self.max_height:
                z = 0.2
                x = 0
                action = [-x,0,z,0,0,0]
                self.simulation.give_vel(action,Time = 0.5)
            else:
                z = -0.1
                action = [0,0,z,0,0,0]
                self.simulation.give_vel(action,Time = 0.2)
                break
            t = t + 1
            if t>5:
                break

        self.simulation.give_vel([0,0,0,0,0,0],Time = 0.2)
        print('Reset Successful')
        self.state,_,_ = self.step([0, 0, 0, 0, 0, 0], 2)

        return self.state

    def setReward(self,state,done,action):

        avoid = []
        ag = []
        act_reward = []
        for i in range(5):
            angle = math.pi / 5 - self.heading - (math.pi / 10 * i) 
            ag.append(angle)
            tr = 0.75 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * (angle + math.pi / 2) % (2 * math.pi) / math.pi)[0])
            act_reward.append(tr)    

        obs_min = min(self.current_info['lidar'])
            
        if obs_min < 0.3:
            avoid = -10
        else:
            avoid = 0

        if done :

            if self.dist < self.goal_range:   
                self.reward = 1000 
                print('achieve goal')
            else:
                self.reward = -500
        else:
            distance_rate = 2**(self.dist/0.6)
            self.reward = round(((act_reward[action] * 5) * distance_rate), 2) + avoid

        return self.reward
        












