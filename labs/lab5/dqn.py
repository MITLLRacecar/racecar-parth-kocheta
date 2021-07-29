import numpy as np
import random
import os
import torch
import torch.nn as nn





class Dqn:
    def __init__(self):
        self.last_action = 0
        self.last_reward = 0
        self.last_state = []
        self.memory = ReplayMemory(100000)

    def choose_action(self, state):

    def update(self, reward, new_state):
        self.memory.push(last_)
    
