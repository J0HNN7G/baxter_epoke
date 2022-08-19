#!/usr/bin/env python
# kinetic

#!/usr/bin/env python3
# noetic


# Steps to run this code
# 1) roslaunch baxter gazebo launch script
# 2) roslaunch baxter_moveit_tutorial init_moveit.launch
# 3) rosrun baxter_moveit_tutorial main.py


# general
import sys
import numpy as np

# ros/moveit
import rospy
import moveit_commander
import geometry_msgs.msg

# environment/robot/agent
from src.environment import Environment
from src.baxter import Baxter
import src.utils


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


class SceneInterface():
    """
    Wrapper class for MoveIt PlanningSceneInterface for cleaner
    use when adding objects
    """

    # scene which MoveIt is using for planning
    scene = None


    def __init__(self):
        """Do not want unintended objects during initialization"""
        pass


    def setupScene(self):
        """Set up the scene for collision avoidant motion planning"""
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        rospy.sleep(0.5)

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        p.pose.position.x = 0.95
        p.pose.position.y = 0
        p.pose.position.z = -0.55
        scene.add_box('table', p, (1.3, 1.3, 0.73))

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        p.pose.position.x = 0.6
        p.pose.position.y = 0
        p.pose.position.z = -0.55 + 0.73 / 2 + 0.07 / 2
        scene.add_box('box', p, (0.22, 0.31, 0.07))

        self.scene = scene


    def endScene(self):
        """Remove all objects from the MoveIt scene"""

        rospy.sleep(0.25)

        for name in self.scene.get_objects().keys():
            self.scene.remove_world_object(name)


# ------------------------------ Default parameters ------------------------------

LEARNING_RATE = 0.1
DISCOUNT = 0.95
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
    success = False
    while not success:
        x = src.utils.prop2norm(np.random.rand())
        y = src.utils.prop2norm(np.random.rand())
        t = src.utils.prop2norm(np.random.rand())
        l = np.random.rand()

        success = robot.doPoke(x,y,t,l)

    upOutcome, restOutcome = robot.doReset()


if __name__ == '__main__':
    try:
        # rospy + MoveIt
        setupNode()
        sceneInterface = SceneInterface()
        sceneInterface.setupScene()

        # environment
        env = Environment()
        env.cube.resetPose(0.5,0.75,0.25)

        # body
        robot = Baxter()
        robot.start()

        # agent
        agent = None # TODO

        main(env, robot, agent)

        # clean up everything (kill topics, unallocate resources,
        # remove scene objects)
        robot.stop()
        sceneInterface.endScene()
        endNode()
    except rospy.ROSInterruptException:
        pass
