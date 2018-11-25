# -*- coding: utf-8 -*-
"""
Created on Thu Nov  8 14:38:32 2018

@author: overs
"""
import time
import numpy as np
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist, Point, Pose
from rl_control import State_feedback
from collections import defaultdict
import rospy
class Env(object):
    def __init__(self):
        '''
        [forward,left,backward,right,up,down] = [0,1,2] 
        
        '''
        self.threshold = 0.1 # check if collision
        self.simulation  = State_feedback()
        self.state  = (0,0,0)
        self.state_space = None
        #self.action_space = [0,1,2,3] #?
        self.initial_pose = (0,0,0.2)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.current_info = defaultdict(list)
        self.Goalstate = (0,0,1)
        self.reward = 0
    #def Available_action(self):
        
        
        
    def step(self,action):
        '''
        Input: 1 x 3 List
        Output:Current_state,current_reward,current_status,{}
        publish the action to gazebo model and return it in several second
        '''
        vel = action + [0,0,0] # only consider linear velocity
        self.simulation.give_vel(vel  ,Time = 1)
        '''
        while self.simulation.command:
            continue
        '''    
        ###
        Info =  self.Get_current_info()
        self.state = Info['Current_Pose'][:3]
        self.Lidar = Info['lidar']
        done =self.Done_check()
        reward = self.setReward(self.state,done,action)
        ###
        Status = done
         
        
        return self.state,reward,Status,{}
    
    def Get_current_info(self):
        '''
        Get Current information, which contains: Position,Velocity, Lidar Data
        '''        
        Current_Pose = self.simulation.pos
        Current_velocity = self.simulation.vel
        Lidar_data = self.simulation.lidar_points
        self.current_info['Current_Pose'] = Current_Pose
        self.current_info['Current_velocity'] = Current_velocity
        self.current_info['lidar'] = Lidar_data
        
        
        
        
        
        return self.current_info
    def abs_opr(self,x):
        return sum(abs(i) for i in x)
    def Done_check(self):
        '''
        Check if the quadrotor make collision or achieve target.
        '''
        ##
        Min_dist = min(list(map(self.abs_opr,self.Lidar)))

        if Min_dist <= self.threshold:
            print(3)
            return True
        ##
        elif sum(np.array(self.Goalstate) - np.array(self.state)) < 0.01 :
            print(4)
            return True
            
        else:
            return False
        
    def reset(self):
        '''
        reset env to orginal position [0,0,0]
        
        '''
        self.reward = 0
        self.simulation.set_position(self.initial_pose)
        Time = time.clock()
        print('Reseting...')
        '''
        while   self.simulation.pos[:3] != self.initial_pose and time.clock() - Time <5:
            #print(self.simulation.pos[:3])
            continue
        print(self.simulation.pos[:3])
        if self.simulation.pos[:3] == self.initial_pose:
            print('Reset Successful')
            self.state =  self.initial_pose[:]
        else:
            print('Reset fail, try again...')
        '''

    def setReward(self,state,done,action):
        if done :
            if state == self.Goalstate:
                self.reward += 1000 
            else :
                self.reward -= 1000
        else:
            self.reward -= 1
            
        
        
        return self.reward
        











