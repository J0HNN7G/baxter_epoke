#!/usr/bin/env python
# kinetic

#!/usr/bin/env python3
# noetic


# general
import sys
import numpy as np

# plotting
from tqdm import tqdm #used for loading bars
import matplotlib.pyplot as plt

# ros/moveit
import rospy
import moveit_commander

# environment/robot
import src.utils
from src.environment import Environment
from src.baxter import Baxter

# agent
import gym
from src.model.q_agent import QAgent


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
    #moveit_commander.os._exit(0)


# ------------------------------ Default parameters ------------------------------

# number of episodes
EPISODES = 5

# learning rate
ALPHA = 0.1

# discount
GAMMA = 0.95

# Exploration settings
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = EPISODES//2
epsilon = 1.0  # not a constant, qoing to be decayed
epsilon_decay_value = epsilon/(END_EPSILON_DECAYING - START_EPSILON_DECAYING)

# state splits
WIDTH_SPLIT = 8
HEIGHT_SPLIT = 8
QUAT_SPLIT = 8
STATE_SPLITS = np.array([WIDTH_SPLIT, HEIGHT_SPLIT] + [QUAT_SPLIT] * 4) 

# action splits
THETA_SPLIT = 8
LENGTH_SPLIT = 3
ACTION_SPLITS = np.array([WIDTH_SPLIT, HEIGHT_SPLIT, THETA_SPLIT, LENGTH_SPLIT])


# ------------------------------ Main ------------------------------

def main(env, robot, agent):

    total_reward = 0
    ep_rewards = []

    print('\n\n')
    for episode in tqdm(range(0, EPISODES)):
        rospy.loginfo('\n\n\nEPISODE ' + str(episode))

        # robot full reset
        env.endSceneMoveIt()
        robot.setupArmPoses()
        env.setupSceneMoveIt()

        env.reset()
        rospy.sleep(0.1)

        episodic_return = 0

        obs = env.getDiscreteState()
        while not env.done():
            discreteAction = agent.act(obs)
            action = src.utils.discPoke2norm(discreteAction, ACTION_SPLITS)
            rospy.loginfo('\n\nACTION ' + str(env.num_actions))

            pokeOutcomes = robot.doPoke(*action)
            if not all(pokeOutcomes):
                break
            else:
                env.num_actions += 1

            #env.addCubeMoveIt()
            resetOutcomes = robot.doReset()
            #env.removeCubeMoveIt()
            if not all(resetOutcomes):
                break

            # run into issue where the cube is still moving, wait till it stops before getting reward?
            new_obs, reward = env.getDiscreteObservation()
            agent.learn(obs, action, reward, new_obs)

            obs = new_obs
            episodic_return += reward

        total_reward += episodic_return
        ep_rewards.append(episodic_return)

        if END_EPSILON_DECAYING > episode >= START_EPSILON_DECAYING:
            agent.epsilon -= epsilon_decay_value
        print('\n\n')

    return total_reward, agent.q_table, ep_rewards


if __name__ == '__main__':
    try:
        # rospy + MoveIt
        setupNode()

        # environment
        env = Environment(STATE_SPLITS)
        env.setupSceneMoveIt()

        # body
        robot = Baxter()
        robot.start()

        # agent
        agent = QAgent(
            action_space=gym.spaces.MultiDiscrete(ACTION_SPLITS),
            obs_space=gym.spaces.MultiDiscrete(STATE_SPLITS),
            gamma=GAMMA,
            alpha=ALPHA,
            epsilon=epsilon
        )

        total_reward, q_table, ep_rewards = main(env, robot, agent)
        print("Total reward over training: {}\n".format(total_reward))
        plt.plot(ep_rewards)
        plt.show()

        # clean up everything (kill topics, unallocate resources,
        # remove scene objects)
        robot.stop()
        env.endSceneMoveIt()
        endNode()
    except rospy.ROSInterruptException:
        pass
