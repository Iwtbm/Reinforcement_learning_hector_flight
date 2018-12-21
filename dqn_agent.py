#!/usr/bin/env python

import tensorflow as tf
import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32MultiArray
from Env import Env
from tensorflow.keras.models import Sequential, load_model
from tensorflow.keras.optimizers import RMSprop
from tensorflow.keras.layers import Dense, Dropout, Activation
import math
EPISODES = 3000


class ReinforceAgent():
    def __init__(self, state_size, action_size):
        self.dirPath = os.path.dirname(os.path.realpath(__file__))

        self.load_model = True               ## load model
        self.load_episode = 700              ## load ep
        self.state_size = state_size
        self.action_size = action_size
        self.episode_step = 6000
        self.target_update = 2000
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 0.05                   ## load eps
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 500
        self.memory = deque(maxlen=1000000)
        self.model = self.buildModel()
        self.target_model = self.buildModel()

        self.updateTargetModel()

        if self.load_model:
            self.model.set_weights(load_model(self.dirPath + str(self.load_episode) + ".h5").get_weights())

    def buildModel(self):
        model = Sequential()
        dropout = 0.2

        model.add(Dense(64, input_shape=(self.state_size,), activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dense(64, input_shape=(self.state_size,), activation='relu', kernel_initializer='lecun_uniform'))

        model.add(Dropout(dropout))

        model.add(Dense(self.action_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        model.summary()

        return model

    def getQvalue(self, reward, next_target, done):
        if done:
            return reward
        else:
            return reward + self.discount_factor * np.amax(next_target)

    def updateTargetModel(self):
        self.target_model.set_weights(self.model.get_weights())

    def getAction(self, state):
        if np.random.rand() <= self.epsilon:
            self.q_value = np.zeros(self.action_size)
            return random.randrange(self.action_size)
        else:
            q_value = self.model.predict(state.reshape(1, len(state)))
            self.q_value = q_value
            return np.argmax(q_value[0])

    def appendMemory(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def trainModel(self, target=False):
        mini_batch = random.sample(self.memory, self.batch_size)
        X_batch = np.empty((0, self.state_size), dtype=np.float64)
        Y_batch = np.empty((0, self.action_size), dtype=np.float64)

        for i in range(self.batch_size):
            states = mini_batch[i][0]
            actions = mini_batch[i][1]
            rewards = mini_batch[i][2]
            next_states = mini_batch[i][3]
            dones = mini_batch[i][4]

            q_value = self.model.predict(states.reshape(1, len(states)))
            self.q_value = q_value

            if target:
                next_target = self.target_model.predict(next_states.reshape(1, len(next_states)))

            else:
                next_target = self.model.predict(next_states.reshape(1, len(next_states)))

            next_q_value = self.getQvalue(rewards, next_target, dones)

            X_batch = np.append(X_batch, np.array([states.copy()]), axis=0)
            Y_sample = q_value.copy()

            Y_sample[0][actions] = next_q_value
            Y_batch = np.append(Y_batch, np.array([Y_sample[0]]), axis=0)

            if dones:
                X_batch = np.append(X_batch, np.array([next_states.copy()]), axis=0)
                Y_batch = np.append(Y_batch, np.array([[rewards] * self.action_size]), axis=0)

        self.model.fit(X_batch, Y_batch, batch_size=self.batch_size, epochs=1, verbose=0)

if __name__ == '__main__':
    rospy.init_node('dqn_agent')
    speed = 0.15
    pi = math.pi
    theta = [-pi/2, -pi/4, 0, pi/4, pi/2]
    state_size = 32
    action_size = 5

    env = Env()

    agent = ReinforceAgent(state_size, action_size)
    scores, episodes = [], []
    global_step = 0
    start_time = time.time()
    Status = False
    for e in range(agent.load_episode + 1, EPISODES):
        done = False
        state = env.reset()
        score = 0
        t = 0

        while not done and t <= agent.episode_step :
            st = time.time()
            Status = rospy.is_shutdown()
            resample = []
            
            if Status:
                break
            action = agent.getAction(state)
            vel = [speed, 0, 0, 0, 0, theta[action]]
            next_state, reward, done = env.step(vel, action)
            if e > 706:
                agent.appendMemory(state, action, reward, next_state, done)

            if len(agent.memory) >= agent.train_start:
                if global_step <= agent.target_update:
                    agent.trainModel()
                else:
                    agent.trainModel(True)

            score += reward
            state = next_state

            if e % 100 == 0 and done:
                agent.model.save(agent.dirPath + str(e) + '.h5')
                print('Model Saved')

            if t >= 300:
                rospy.loginfo("Time out!!")
                done = True

            if done:
                agent.updateTargetModel()
                scores.append(score)
                episodes.append(e)
                m, s = divmod(int(time.time() - start_time), 60)
                h, m = divmod(m, 60)

                rospy.loginfo('Ep: %d score: %d memory: %d epsilon: %.2f time: %d:%02d:%02d',
                              e, score, len(agent.memory), agent.epsilon, h, m, s)
                break
            
            global_step += 1
            if global_step % agent.target_update == 0:
                rospy.loginfo("UPDATE TARGET NETWORK")
            t += 1
            
        if Status:
            break

        if agent.epsilon > agent.epsilon_min:
            agent.epsilon *= agent.epsilon_decay