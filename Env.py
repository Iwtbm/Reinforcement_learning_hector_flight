#!/usr/bin/env python

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
        print(self.simulation.pos)
        self.state  = (0,0,0)
        self.state_space = None
        #self.action_space = [0,1,2,3] #?
        self.initial_pose = (0,0,0.2)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.current_info = defaultdict(list)
        # self.Goalstateself.Goalstate = (0,1,1)
        self.reward = 0
        self.max_height = 1
    #def Available_action(self):    
        
        
    def step(self,action):
        '''
        Input: 1 x 6 List
        Output:Current_state,current_reward,current_status,{}
        publish the action to gazebo model and return it in several second
        '''
        vel = action 
        self.simulation.give_vel(vel,Time = 1)
        '''
        while self.simulation.command:
            continue
        '''    
        ###
        Info =  self.Get_current_info()
        dist = np.sqrt(sum(np.square((Info['Current_Pose'][:3]))))
        heading = Info['obs_max'] - Info['lidar'][541]
        max_index = Info['lidar'].index(Info['obs_max'])
        if Info['obs_max'] - Info['lidar'][max_index + 5] >= 0.3:
            i = 0
            while True:
                i = i + 1 
                if Info['obs_max'] - Info['lidar'][max_index - i] >= 0.3:
                    angle = i * 0.04
                    break
        else:
            i = 0
            while True:
                i = i + 1 
                if Info['obs_max'] - Info['lidar'][max_index + i] >= 0.3:
                    angle = i * 0.04
                    break

        self.state = Info['lidar'] + dist + Info['obs_max'] + Info['obs_min'] + heading + angle
        # self.Lidar = Info['lidar']
        done =self.Done_check()
        reward = self.setReward(self.state,done,action)
        ###
        status = done
         
        return self.state, reward, status
        # return self.state,reward,Status,{}

    
    def Get_current_info(self):
        '''
        Get Current information, which contains: Position,Velocity, Lidar Data
        '''        
        self.current_info['Current_Pose'] = self.simulation.pos
        self.current_info['Current_velocity'] = self.simulation.vel
        self.current_info['lidar'] = self.simulation.laser_ranges
        self.current_info['obs_max'] = self.simulation.obs_max
        self.current_info['obs_min'] = self.simulation.obs_min
        
        return self.current_info


    def abs_opr(self,x):
        
        return sum(abs(i) for i in x)


    def Done_check(self):
        '''
        Check if the quadrotor make collision or achieve target.
        '''
        ##
        Min_dist = self.current_info['obs_min']

        if Min_dist <= self.threshold:
            #print(3)
            return True
        ##
        # elif sum(np.array(self.Goalstate) - np.array(self.state)) < 0.01 :
        #     #print(4)
        #     return True
            
        else:
            return False

        
    def reset(self):
        '''
        reset env to orginal position [0,0,0]
        
        '''
        self.reward = 0
        self.simulation.set_position(self.initial_pose)
        # Time = time.clock()
        print('Reseting...')
        while True and not rospy.is_shutdown():
            if self.state[2] < self.max_height:
                z = 0.1
                action = [0,0,z,0,0,0]
                self.simulation.give_vel(action,Time = 1)
            else:
                break
        print('Reset Successful')
        self.state = self.step([0, 0, 0, 0, 0, 0])
        return self.state
        # '''
        # while   self.simulation.pos[:3] != self.initial_pose and time.clock() - Time <5:
        #     #print(self.simulation.pos[:3])
        #     continue
        # print(self.simulation.pos[:3])
        # if self.simulation.pos[:3] == self.initial_pose:
        #     print('Reset Successful')
        #     self.state =  self.initial_pose[:]
        # else:
        #     print('Reset fail, try again...')
        # '''

    def setReward(self,state,done,action):
        self.reward = self.reward + (state[-1] * 5)**2 - 5 * state[-2] - 1/state[-3] + 2 * state[-4] + 3 * state[-3]
        if done :
            # if state == self.Goalstate:
            #     self.reward += 1000 
            # else :
            self.reward -= 1000
        else:
            self.reward -= 1
            
        
        
        return self.reward
        












