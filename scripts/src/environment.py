#!/usr/bin/env python


# general
import rospy
import numpy as np

# cube
import std_msgs.msg
import geometry_msgs.msg
import gazebo_msgs.msg
from scipy.spatial.transform import Rotation

# converters
from . import utils


# ------------------------------ Constants ------------------------------

# height of cube center (in metres) for Gazebo world frame
GAZEBO_CUBE_HEIGHT = 0.825
# height of cube floor (in metres) for Gazebo world frame
GAZEBO_BOX_HEIGHT = 0.8
# name of cube link from message in /gazebo/link_states topic
GAZEBO_CUBE_LINK_NAME = 'cube::cube_link'


# ------------------------------ Environment classes ------------------------------

class Environment():
    """Class for agent environment"""

    # cube object that the agent is poking
    cube = None

    def __init__(self):
        """Initialize cube publisher and subscriber object"""
        self.cube = Cube()


    def getObservation(self):
        """
        Get x,y of centre and orientation of object normalized to [-1,1]

        return: [x, y, qx, qy, qz, qw]
        """
        pose = self.cube.getPose()
        position = [ utils.width2norm(pose.position.x), utils.height2norm(pose.position.y)]
        orientation = [ utils.angle2norm(a) for a in  Rotation.from_quat( [ pose.orientation.x,
                                                                      pose.orientation.y,
                                                                      pose.orientation.z,
                                                                      pose.orientation.w]).as_euler('ZYX')]
        return np.concatenate(position, orientation)


    def resetEnvironment(self):
        """reset environment for new learning episode"""
        self.cube.resetPose(0,0,0)


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
        self.subCube = GazeboLinkPose(GAZEBO_CUBE_LINK_NAME)
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

        pos_vel = [x, y, GAZEBO_CUBE_HEIGHT, yaw]
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


    def hasFallen(self):
        """
        Check if cube has fallen from workspace

        return: True if cube has fallen from workspace, else False
        """
        return self.subCube.link_pose.position.z < GAZEBO_BOX_HEIGHT


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
