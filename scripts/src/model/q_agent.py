

import itertools
import collections 
import numpy as np
import gym


class QAgent():
    # Simple class for Q-Learning agent

    def __init__(self, action_space, obs_space, alpha, gamma, epsilon, seed=42):
        # type: (Space, Space, float, float, float) -> None
        self.action_space = action_space
        self.obs_space = obs_space

        if isinstance(self.action_space, gym.spaces.Discrete):
            self.all_actions = list(range(action_space.n))
        else:
            self.all_actions = [ np.array(element).tobytes() for element in itertools.product(
                                    *[range(n) for n in action_space.nvec])]

        self.epsilon = epsilon
        self.gamma = gamma
        self.alpha = alpha

        # table for Q-values mapping (OBS, ACT) pairs of observations and actions to respective Q-values
        self.q_table = collections.defaultdict(lambda: 0)

        np.random.seed(seed)
        

    """
    :param obs (int): received observation representing the current environmental state
    :return (int): index of selected action
    """
    def act(self, obs):
        # type: [int] -> [int]

        #epsilon-greedy action selection
        if np.random.random() < self.epsilon: # TODO figure this out
            return self.action_space.sample()
        else:
            if isinstance(self.action_space, gym.spaces.Discrete):
                return np.argmax( [ self.q_table[(obs, a )] for a in self.all_actions])
            else: 
                obs = obs.tobytes()
                action_qs = [ self.q_table[(obs, a)] for a in self.all_actions]
                idx = np.random.choice(np.flatnonzero(action_qs == np.max(action_qs)))
                return np.frombuffer(self.all_actions[idx], dtype=int)


    def learn(self, obs, action, reward, n_obs):
        # type: ([int], [int], float, [int]) -> float
        if isinstance(self.action_space, gym.spaces.Discrete):
            max_future_q = np.max( [ self.q_table[(n_obs, a)] for a in self.all_actions])
            self.q_table[(obs, action )] += self.alpha * (reward + self.gamma * max_future_q - self.q_table[(obs, action)])
            return self.q_table[(obs, action)]
        else: 
            obs = obs.tobytes()
            action = action.tobytes()
            n_obs = n_obs.tobytes()
            max_future_q = np.max( [ self.q_table[(n_obs, a)] for a in self.all_actions])
            self.q_table[(obs, action )] += self.alpha * (reward + self.gamma * max_future_q - self.q_table[(obs, action)])
            return self.q_table[(obs, action)]