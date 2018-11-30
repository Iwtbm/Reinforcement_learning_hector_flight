#!/usr/bin/env python

import numpy as np
import rospy
from Env import Env
import random

class random_agent(object):
    def __init__(self):
		
        self.Env = Env()
	
    def rdn_agent(self,max_height):
        self.Env.reset()
        self.state = (0,0,1)
        while True:
            if self.state[2] < max_height:
                z = 0.1
                action = [0,0,z,0,0,0]
            else:
                x = 0.1
               # theta_list = [0,0.75,1.5,-0.75,-1.5] 
               # rdm = np.random.randint(0,5)
               # theta = theta_list[rdm]  
                theta = 1.5 	    
    	        action = [x,0,0,theta,0,0]
            self.state,reward,Status,a = self.Env.step(action)
            print(self.state)
            print(reward)
            print(Status)
            if Status == True:
                self.Env.reset()
				
if __name__ == '__main__':
    rospy.init_node('agent', anonymous=True)
    agt = random_agent()
    agt.rdn_agent(1)
    while not rospy.is_shutdown():
        rospy.spin()	
			
		
