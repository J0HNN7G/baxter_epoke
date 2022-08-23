#!/usr/bin/env python


import numpy as np
from .. import utils


class RandomAgent:
	"""Random agent that ignores state for decided actions"""

	def __init__(self):
		"""No state consideration, so nothing to do here"""
		pass


	def learn(self, before, action, after, reward):
		"""Agent does not learn"""
		pass


	def act(self, obs):
		"""
		Agent returns random action

		return: [x,y,t,l]; x is poke centre x-axis value (in metres);
        y is poke centre y-axis value (in metres); t is angle of 
        poke (in radians) w.r.t to baxter base frame; l is length of 
        poke (in metres)
        """
		return utils.randomAction()