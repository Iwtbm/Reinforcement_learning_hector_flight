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
                c = random.random()
            else:
                c = 0	
    	    a = -0.5
    	    b = -0.5
    	    
    	    action = [a,b,c]
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
			
		
