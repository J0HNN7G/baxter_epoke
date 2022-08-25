#!/usr/bin/env python


# general
import rospy
import numpy as np

# ROS/moveit
import rospy
import moveit_commander

# cube
import std_msgs.msg
import geometry_msgs.msg
import gazebo_msgs.msg
from scipy.spatial.transform import Rotation

# converters
from . import utils


# ------------------------------ Constants ------------------------------

# Maximum amount of time (in seconds) after actions have been committed at which a reset will be done
MAX_AFTER_TIME = 30
# Maximum number of actions in an episode
MAX_ACTIONS = 6

# failure for reward
FAIL_REWARD = -10

# height of cube center (in metres) for Gazebo world frame
GZ_CUBE_HEIGHT = 0.825
# height of cube floor (in metres) for Gazebo world frame
GZ_BOX_HEIGHT = 0.8
# name of cube link from message in /gazebo/link_states topic
GZ_CUBE_LINK_NAME = 'cube::cube_link'
# gazebo cube length (in metres)
GZ_CUBE_LENGTH = 0.05
# rviz moveit reference frame in z-axis for cube center (in metres)
RVIZ_CUBE_Z = (-0.55 + 0.73 / 2 + 0.07 / 2 + 0.05)
# Gazebo to MoveIt Rviz difference in z-axis for world frame
GZ_RVIZ_DIFF_Z = -0.923990705439


# ------------------------------ Environment classes ------------------------------

class Environment:
    """Class for agent environment and MoveIt scene wrapper"""

    # Maximum number of max_actions
    max_actions = -1

    # number of actions taken by agent in episode
    num_actions = -1

    # maximum amount of time after action for episode
    max_time = -1

    # start_time of current episode (in seconds)
    start_time = -1

    # reward for failure
    FAIL_REWARD = -10

    # cube object that the agent is poking
    cube = None

    # MoveIt robot commander for scene frame
    robot = None

    # scene which MoveIt is using for planning
    scene = None


    def removeCubePose(self):
        """Remove cube from MoveIt scene"""
        self.scene.remove_world_object('cube')


    def __init__(self, max_actions=MAX_ACTIONS, max_time=MAX_AFTER_TIME):
        """Initialize episode parameters, cube publisher, cube subscriber"""
        self.max_actions = max_actions
        self.max_time = max_time
        self.cube = Cube()
        self.num_pokes = 0
        self.start_time = rospy.get_time()


    def reset(self):
        """Reset environment for new learning episode"""
        self.cube.resetPose(0,0,0)
        self.num_actions = 0
        self.start_time = rospy.get_time()


    def done(self):
        """
        Check if current world episode is done

        return: True if cube has fallen, number of actions or time exceeds maximum
        """
        return self.cubeHasFallen() or (self.num_actions >= self.max_actions) or (self.getElapsedTime() > self.max_time)


    def calcState(self, pose):
        """
        Get x,y of centre and orientation of cube normalized to [-1,1]

        pose: geometry_msgs.msg for cube link pose
        return: [x, y, qx, qy, qz, qw]
        """
        position = [ utils.width2norm(pose.position.x), utils.height2norm(pose.position.y)]
        orientation = [ utils.angle2norm(a) for a in  Rotation.from_quat( [ pose.orientation.x,
                                                                      pose.orientation.y,
                                                                      pose.orientation.z,
                                                                      pose.orientation.w]).as_euler('ZYX')]
        return np.concatenate((position, orientation))


    def calcReward(self, pose):
        """
        Calculate reward from cube link pose

        return: fail reward if cube has fallen, otherwise absolute
                distance from center of workspace
        """
        if pose.position.z < GZ_BOX_HEIGHT:
            return FAIL_REWARD
        else:
            reward = np.abs(utils.width2norm(pose.position.x)) \
                   + np.abs(utils.height2norm(pose.position.y))
            return reward


    def getState(self):
        """
        Get x,y of centre and orientation of object normalized to [-1,1]

        return: [x, y, qx, qy, qz, qw]
        """
        pose = self.cube.getPose()
        return self.calcState(pose)


    def getReward(self):
        """
        Get current reward from environment

        return: fail reward if cube has fallen, otherwise absolute
                distance from center of workspace
        """
        pose = self.cube.getPose()
        return calcReward(pose)


    def getObservation(self):
        """
        Wrapper for getting state and reward in one function

        return: [x, y, qx, qy, qz, qw], fail reward if cube has fallen,
                otherwise absolute distance from center of workspace
        """
        pose = self.cube.getPose()
        return self.calcState(pose), self.calcReward(pose)


    def getElapsedTime(self):
        """
        Get episode time elapsed (in seconds)

        return: episode time (in seconds)
        """
        return rospy.get_time() - self.start_time


    def cubeHasFallen(self):
        """
        Check if cube has fallen from workspace

        return: True if cube has fallen from workspace, else False
        """
        return self.cube.subCube.link_pose.position.z < GZ_BOX_HEIGHT


    def setupSceneMoveIt(self):
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

        self.robot = robot
        self.scene = scene


    def endSceneMoveIt(self):
        """Remove all objects from the MoveIt scene"""
        rospy.sleep(0.25)

        for name in self.scene.get_objects().keys():
            self.scene.remove_world_object(name)


    def addCubeMoveIt(self):
        """
        Add cube to MoveIt scene

        pose: geometry_msgs.msg pose for cube link
        """
        cubePose = geometry_msgs.msg.PoseStamped()
        cubePose.header.frame_id = self.robot.get_planning_frame()
        cubePose.pose = self.cube.subCube.link_pose
        cubePose.pose.position.z = cubePose.pose.position.z + GZ_RVIZ_DIFF_Z
        self.scene.add_box('cube', cubePose, (GZ_CUBE_LENGTH,) * 3)


    def removeCubeMoveIt(self):
        """
        Remove cube from MoveIt scene
        """
        self.scene.remove_world_object('cube')


class Cube:
    """
    Cube publisher and subscriber wrapper class for easily setting cube properties
    and getting them
    """

    # cube pose and velocity publisher
    pubCube = None

    # cube pose subscriber wrapped in helpful class
    subCube = None


    def __init__(self):
        """Create cube position and velocity publisher and pose subscriber"""
        self.pubCube = rospy.Publisher("/cube/position_velocity_cmd", std_msgs.msg.Float32MultiArray, queue_size=10)
        self.subCube = GazeboLinkPose(GZ_CUBE_LINK_NAME)
        rospy.sleep(0.5)


    def getPose(self):
        """
        Get geometry_msgs.msg cube pose

        return: cube pose from rostopic subscriber
        """
        return self.subCube.link_pose


    def resetPose(self, x, y, yaw):
        """
        Reset pose for cube in workspace

        x: x-axis center position of cube in wspace normalized ([-1,1])
        y: y-axis center position of cube in wspace normalized ([-1,1])
        yaw: yaw of cube normalized ([-1,1] == [-pi, pi])
        """
        x = utils.norm2width(x)
        y = utils.norm2height(y)
        yaw = utils.norm2angle(yaw)

        pos_vel = [x, y, GZ_CUBE_HEIGHT, yaw]
        pos_vel_msg = std_msgs.msg.Float32MultiArray(data=pos_vel)
        self.pubCube.publish(pos_vel_msg)
        rospy.sleep(0.1)


    def setPose(self, x, y, z, qx, qy, qz, qw):
        """
        Set pose of cube in world frame

        x: x-axis center of cube in world frame
        y: y-axis center of cube in world frame
        z: z-axis center of cube in world frame
        qx: x-value of orientation quaternion
        qx: y-value of orientation quaternion
        qx: z-value of orientation quaternion
        qw: real value for orientation quaternion
        """
        pos_vel = [x, y, z, qx, qy, qz, qw]
        pos_vel_msg = std_msgs.msg.Float32MultiArray(data=pos_vel)
        self.pubCube.publish(pos_vel_msg)
        rospy.sleep(0.1)


class GazeboLinkPose:
  """
  Credit to Boris Gromov:
  https://github.com/bgromov/thymio_gazebo/blob/master/thymio_description/scripts/gazebo_link_pose

  Class for getting pose information from Gazebo links
  """

  # name of gazebo link in rostopic
  link_name = ''

  # link pose being updated from rostopic
  link_pose = geometry_msgs.msg.Pose()

  def __init__(self, link_name):
    """
    Create link pose subscriber using link_name

    link_name: name of link whose pose is of interest
               in /gazebo/link_states rostopic
    """
    self.link_name = link_name

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates, self.callback, queue_size=1)


  def callback(self, data):
    """
    function that periodically updates link_pose using data from link_states subscriber

    data: gazebo_msgs.msg.LinkStates including desired link pose information
    """
    try:
      ind = data.name.index(self.link_name)
      self.link_pose = data.pose[ind]
    except ValueError:
      pass
