import numpy as np
import random
import os
import torch
import torch.nn as nn




class ReplayMemory(object):
    
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        
    def push(self, event):
        self.memory.append(event)
        if len(self.memory) > self.capacity:
            del self.memory[0]
            
    def sample(self, batch_size):
        samples = zip(*random.sample(self.memory, batch_size))

class Dqn:
    def __init__(self):
        self.last_action = 0
        self.last_reward = 0
        self.last_state = []
        self.memory = ReplayMemory(100000)

    def choose_action(self, state):

    def update(self, reward, new_state):
        self.memory.push(last_)
    
