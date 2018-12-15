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

class Env(object):
    def __init__(self):
        '''
        [forward,left,backward,right,up,down] = [0,1,2] 
        
        '''
        self.threshold = 0.7 # check if collision
        self.simulation  = State_feedback()
        # print(self.simulation.pos)
        self.state  = []
        self.state_space = None
        #self.action_space = [0,1,2,3] #?
        self.initial_pose = (0,0,0.19)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.current_info = defaultdict(list)
        # self.Goalstateself.Goalstate = (0,1,1)
        self.reward = 0.0
        self.max_height = 0.5
        self.goal = [0,2]
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
        '''
        if Info['obs_max'] - Info['lidar'][min(max_index + 5,1080)] >= 0.3:
            i = 0
            while True:
                i = i + 1
                if max_index - i <= 1:
                    break 
                if Info['obs_max'] - Info['lidar'][max_index - i] >= 0.3:
                    angle = i * 0.04
                    break
        else:
            i = 0
            while True:
                i = i + 1 
                if max_index + i >= 1079:
                    break
                if Info['obs_max'] - Info['lidar'][max_index + i] >= 0.3:
                    angle = i * 0.04
                    break
        '''
        # print(Info['lidar'].type)
        #self.state = list(Info['lidar']) + [dist, Info['obs_max'], Info['obs_min'], heading, angle]
        ### self.state = list(Info['lidar']) + [dist, Info['obs_min']]
        theta_pos = tf.transformations.euler_from_quaternion(Info['position'][-4:])
        #print(theta_pos)
        #print(Info['position'][:2])
        self.state = list(Info['position'][:2]) + [theta_pos[2]]
        # self.Lidar = Info['lidar']
        print(self.state)
        done =self.Done_check()
        reward = self.setReward(self.state,done,action)
        ###
        status = done
        print(done) 
        return np.asarray(self.state), reward, status
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
        self.current_info['position'] = self.simulation.pos
        
        return self.current_info


    def abs_opr(self,x):
        
        return sum(abs(i) for i in x)


    def Done_check(self):
        '''
        Check if the quadrotor make collision or achieve target.
        '''
        ##
        Min_dist = self.current_info['obs_min']
        print(Min_dist,self.threshold)
        print(self.current_info['position'])
        if Min_dist <= self.threshold:
            # print(self.current_info['lidar'])
            return True

        ##
        elif sum(abs(np.asarray(self.goal) - np.asarray(self.current_info['position'][:2]))) < 1 :
            #print(4)
            return True
            
        else:
            return False

        
    def reset(self):
        '''
        reset env to orginal position [0,0,0]
        
        '''
        self.reward = 0
        Start = time.clock()
        while True:
			self.simulation.set_position(self.initial_pose)
			if time.clock() - Start >=0.3:
				break
       
        # Time = time.clock()
        print('Reseting...')
        while True and not rospy.is_shutdown():
            print(self.simulation.vel)
            if self.simulation.pos[2] < self.max_height:
                z = 0.3
                action = [0,0,z,0,0,0]
                print(self.simulation.pos[2])
                self.simulation.give_vel(action,Time = 1)
            else:
                z = -0.1
                action = [0,0,z,0,0,0]
                print(self.simulation.pos[2])
                self.simulation.give_vel(action,Time = 0.3)
                break
        print('Reset Successful')
        self.state,_,_ = self.step([0, 0, 0, 0, 0, 0])

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
        #self.reward = self.reward + (st1ate[-1] * 5)**2 - 5 * state[-2] - 1/state[-3] + 2 * state[-4] + 3 * state[-3]
        # self.reward = -1/state[-1] * 5 - 1/state[-2] * 10 - sum(np.asarray(self.goal) + 1/np.asarray(self.current_info['position'][:2])) * 10
        print(len(state))
        if done :
             if sum(abs(np.asarray(self.goal) - np.asarray(self.current_info['position'][:2]))) < 1:
                 self.reward = 10000 
                 print('achieve goal')
             else :
                self.reward = -100
        else:
            self.reward = abs(math.atan2(self.goal[1], self.goal[0]) - state[-1])**2 - np.sqrt(sum((np.asarray(self.state[:2]) - np.asarray(self.goal))**2))
            
        
        
        return self.reward
        












