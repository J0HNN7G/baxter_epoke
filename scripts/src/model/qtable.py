#!/usr/bin/env python


import numpy as np


class Qtable:

    q_table = None

    state_splits = None
    state_highs = None
    state_lows = None
    state_incs = None

    action_splits = None
    action_highs = None
    action_lows = None
    action_incs = None


    def __init__(self, state_splits, state_highs, state_lows, action_splits, action_highs, action_lows, init_q_low=-2, init_q_high=0):
        assert (len(state_splits) == len(state_highs) == len(state_lows))
        assert (len(action_splits) == len(action_highs) == len(action_lows))

        self.q_table = np.random.uniform(low=init_q_low, high=init_q_high, size=state_splits+action_splits)
        
        self.state_highs = state_highs
        self.state_lows = state_lows
        self.state_incs = (state_highs - state_lows) / state_splits

        self.action_highs = action_highs
        self.action_lows = action_lows
        self.action_incs = (action_highs - action_lows) / action_splits


    def getDiscreteState(self, state):
        discrete_state = (state - self.state_lows) / self.state_incs
        return tuple(discrete_state.astype(np.int))