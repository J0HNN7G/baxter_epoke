#!/usr/bin/env python
# kinetic

#!/usr/bin/env python3
# noetic


# general
import sys
import numpy as np

# ros/moveit
import rospy
import moveit_commander

# environment/robot/agent
import src.utils
from src.environment import Environment
from src.baxter import Baxter
from src.model.random import RandomAgent


# ------------------------------ ROS/MoveIt ------------------------------

def setupNode():
    """Set up Rospy and MoveIt nodes"""
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_baxter_poke', anonymous=True)


def endNode():
    """End Rospy and MoveIt nodes"""
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


# ------------------------------ Default parameters ------------------------------

LEARNING_RATE = 0.1
DISCOUNT = 0.95

# number of episodes
EPISODES = 20

# Exploration settings
epsilon = 1  # not a constant, qoing to be decayed
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = EPISODES//2
epsilon_decay_value = epsilon/(END_EPSILON_DECAYING - START_EPSILON_DECAYING)

# state splits
WIDTH_SPLIT = 8
HEIGHT_SPLIT = 8
ROLL_SPLIT = 8
PITCH_SPLIT = 8
YAW_SPLIT = 8

# action splits
THETA_SPLIT = 8
LENGTH_SPLIT = 3


# ------------------------------ Main ------------------------------

def main(env, robot, agent):
    for n in range(EPISODES):
        rospy.loginfo('\n\n\nEPISODE ' + str(n))
        env.reset()
        obs = env.getState()

        while not env.done():
            action = agent.act(obs)

            rospy.loginfo('\n\nACTION ' + str(env.num_actions))

            pokeOutcomes = robot.doPoke(*action)
            if not all(pokeOutcomes):
                #tuck_arms(True) TODO
                break
            else:
                env.num_actions += 1

            env.addCubeMoveIt()
            resetOutcomes = robot.doReset()
            env.removeCubeMoveIt()
            if not all(resetOutcomes):
                #tuck_arms(True) TODO
                break

            new_obs, reward = env.getObservation()
            agent.learn(obs, action, new_obs, reward)
            obs = new_obs


if __name__ == '__main__':
    try:
        # rospy + MoveIt
        setupNode()

        # environment
        env = Environment()
        env.setupSceneMoveIt()

        # body
        robot = Baxter()
        robot.start()

        # agent
        agent = RandomAgent()

        main(env, robot, agent)

        # clean up everything (kill topics, unallocate resources,
        # remove scene objects)
        robot.stop()
        env.endSceneMoveIt()
        endNode()
    except rospy.ROSInterruptException:
        pass
