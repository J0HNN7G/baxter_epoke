from collections import defaultdict
import random
from typing import List, Dict, DefaultDict
from gym.spaces import Space
from gym.spaces.utils import flatdim
import numpy as np


class QLearningAgent():
    #Simple class for Q-Learning agent

    def __init__(self, action_space: Space, obs_space: Space, alpha: float, gamma: float, epsilon: float):
        self.action_space = action_space
        self.obs_space = obs_space
        self.n_acts = flatdim(action_space)

        self.epsilon: float = epsilon
        self.gamma: float = gamma #discount factor
        self.alpha: float = alpha

        # table for Q-values mapping (OBS, ACT) pairs of observations and actions to respective Q-values
        self.q_table: DefaultDict = defaultdict(lambda: 0)
        
    """
    :param obs (int): received observation representing the current environmental state
    :return (int): index of selected action
    """
    def act(self, obs: int) -> int:
        #epsilon-greedy action selection
        if random.random() < self.epsilon:
            return np.random.choice([a for a in range(self.n_acts)])
        else:
            return np.argmax([(self.q_table[(obs, a)]) for a in range(self.n_acts)])


    def learn(self, obs: int, action: int, reward: float, n_obs: int, done: bool) -> float:
        max_future_q = np.max([self.q_table[(n_obs, a)] for a in range(self.n_acts)])
        self.q_table[(obs, action)] += self.alpha * (reward + self.gamma * max_future_q - self.q_table[(obs, action)])
        return self.q_table[(obs, action)]