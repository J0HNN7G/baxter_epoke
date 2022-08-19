#!/usr/bin/env python


# general
import copy
import rospy
import numpy as np

# arms
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import tf.transformations

# grippers
import baxter_interface

# converters
import utils


# ------------------------------ Constants ------------------------------

# maximum absolute angle for wrist (in radians), i.e., +- joint limit
WRIST_ABS_MAX = np.pi * 175 / 180

# poke specifications (in metres) in world frame
POKE_MIN_LENGTH = 0
POKE_MAX_LENGTH = 0.05
POKE_HEIGHT = -0.1

# right arm rest pose (Not in use, but useful for world frame values)
RIGHT_ARM_REST_POSE = geometry_msgs.msg.Pose()
RIGHT_ARM_REST_POSE.position.x = utils.WSPACE_CENTRE_X
RIGHT_ARM_REST_POSE.position.y = utils.WSPACE_CENTRE_Y
RIGHT_ARM_REST_POSE.position.z = POKE_HEIGHT + 0.10
RIGHT_ARM_REST_POSE.orientation.x = np.sqrt(2) / 2
RIGHT_ARM_REST_POSE.orientation.y = np.sqrt(2) / 2
RIGHT_ARM_REST_POSE.orientation.z = 0
RIGHT_ARM_REST_POSE.orientation.w = 0

# joint values for both_arms to be in pre-poke poses
REST_JOINT_VALUES = [1.379237576377398, 
                     -1.3552826203873272, 
                     -0.7101624079848046, 
                     2.55631837129957, 
                     0.30652245076879225,
                     0.4115804369116125, 
                     1.3712037894457447, 
                     0.8472704724475024, 
                     -0.8636687059230441, 
                     0.5490456620193518, 
                     1.7160365713575505, 
                     -0.46433543710437775, 
                     0.8569118416172481, 
                     2.387166897523091]

# maximum number of tries when planning with MoveIt
MAX_TRIES = 10


# ------------------------------ Conversion Helpers ------------------------------

def prop2len(value):
    """Convert from [0,1] to length of poke (in metres)"""
    return utils.scale2scale(value,  
                       oMin = 0,                     
                       nMin = POKE_MIN_LENGTH, 
                       nMax = POKE_MAX_LENGTH)


def norm2wrist(value):
    """
    Convert from [-1,1] to [-175,175] degrees (in radians) for calculating 
    points composing poke and how to twist baxter wrist for poke
    """
    return value * WRIST_ABS_MAX


def cm2grip(value, oMin=0.85, oMax=5.0):
    """
    Convert from desired grip width (in centimetres) to grip value for baxter 
    right gripper

    value: desired right gripper grasp width (in cm)
    oMin: right gripper minimum grasp width (in cm)
    oMax: right gripper maximum grasp width (in cm)
    return: remapping of desired gripper grasp (in cm) to corresponding value 
            in gripper interface
    """
    return utils.scale2scale(value, oMin, oMax, 0, 100)


# ------------------------------ MoveIt Helpers ------------------------------

def multiTryPlan(commander, max_tries=MAX_TRIES):
    """
    Attempt to find MoveIt pose target plan multiple times

    commander: MoveIt commander with target pose pre-specified
    max_tries: number of times to try finding a plan
    return: plan if search was successful, else None
    """
    plan = commander.plan()
    num_tries = 1
    while (not plan.joint_trajectory.points) and (num_tries < max_tries):
        plan = commander.plan()
        num_tries += 1

    if not plan.joint_trajectory.points:
        return None
    else:
        return plan


def multiTryCartesianPlan(commander, waypoints, max_tries=MAX_TRIES):
    """
    Attempt to find a MoveIt cartesian trajectory plan multiple times

    commander: MoveIt commander
    waypoints: pose targets for commander to reach sequentially with 
               cartesian trajectory
    max_tries: number of times to try finding a plan
    return: plan if search was successful, else None
    """
    (plan, fraction) = commander.compute_cartesian_path(
        waypoints, # waypoints to follow
        0.005,     # eef_step
        0.0)       # jump_threshold
    num_tries = 1
    while (not validCartesianPlan(plan, fraction)) and (num_tries < max_tries):
        (plan, fraction) = commander.compute_cartesian_path(
            waypoints, # waypoints to follow
            0.005,     # eef_step
            0.0)       # jump_threshold
        num_tries += 1

    if not validCartesianPlan(plan, fraction):
        return None
    else:
        return plan


def validCartesianPlan(plan, fraction):
    """
    Check if MoveIt cartesian plan is successful

    plan: MoveIt cartesian plan
    fraction: proportion of trajectory commander can follow from plan 
    return: True if plan is successful, else False
    """
    return plan.joint_trajectory.points and (fraction == 1)


def slowDownPlan(plan, speed_scale):
    """
    Cartesian trajectory is too fast. This method scales it by a desired 
    factor for a safe execution

    plan: MoveIt cartesian plan
    speed_scale: proportion of speed from the original trajectory plan 
                 that is desired in new plan
    return: cartesian trajectory plan slowed down 
    """
    slow_plan = moveit_msgs.msg.RobotTrajectory()
    slow_plan.joint_trajectory = plan.joint_trajectory
    for i in range(len(plan.joint_trajectory.points)):
        slow_plan.joint_trajectory.points[i].time_from_start = plan.joint_trajectory.points[i].time_from_start / speed_scale
        new_vels = []
        new_accs = []
        new_pos = []
        for j in range(len(plan.joint_trajectory.joint_names)):
            new_vels.append(plan.joint_trajectory.points[i].velocities[j] * speed_scale)
            new_accs.append(plan.joint_trajectory.points[i].accelerations[j] * speed_scale)
            new_pos.append(plan.joint_trajectory.points[i].positions[j])
        slow_plan.joint_trajectory.points[i].velocities = new_vels
        slow_plan.joint_trajectory.points[i].positions = new_pos
        slow_plan.joint_trajectory.points[i].accelerations = new_accs
    return slow_plan


def formatPoke(x, y, theta, length):
    """
    Format poke action into string format

    x: x-axis poke centre (in metres)
    y: y-axis poke centre (in metres)
    theta: poke angle (in radians)
    length: poke length (in metres)
    return: string with poke information
    """
    xm = utils.norm2width(x)
    ym = utils.norm2height(y)
    tm = norm2wrist(theta)
    lm = prop2len(length)
    return "x = {0:3.2f} m - y = {1:3.2f} m - angle = {2:3.2f} deg - length = {3:3.2f} m".format(xm, ym, tm * 180 / np.pi, lm)


def reorientYaw(theta):
    """Make theta (in radians) perpendicular so yaw is as desired during poke"""
    theta += np.pi / 2 
    if theta > WRIST_ABS_MAX:
        theta -= np.pi
    return theta


class Baxter:
    """Class for controlling Baxter robot"""

    # arm controllers
    controls = None

    # gripper controllers
    grippers = None


    def __init__(self):
        """Do not want unintended movement during initialization"""
        pass


    def start(self):
        """Get Baxter ready for executing poke actions"""
        self.setupControls()
        #self.calibrateGrippers() # comment out if need an initial speed up (slow to do)
        self.setupGrippers()
        self.setupArmPoses()


    def stop(self):
        """Shutdown Baxter from executing poke actions"""
        self.stopControls()


    # ------------------------------ Controllers ------------------------------

    def setupControls(self):
        """Set up Baxter MoveIt end-effector commanders"""
        eef_names = ['both_arms', 'right_arm', 'left_arm']
        controls = {}

        for eef in eef_names:
            control = moveit_commander.MoveGroupCommander(eef)
        
            # prevent fast motion
            control.set_max_velocity_scaling_factor(0.5)
            control.set_max_acceleration_scaling_factor(0.3)

            controls[eef] = control
        self.controls = controls


    def stopControls(self):
        """Stop Baxter MoveIt end-effector commanders"""
        for eef in self.controls.values():
            eef.stop()


    # ------------------------------ Grippers ------------------------------

    def calibrateGrippers(self):
        """Calibrate grippers on Baxter"""
        baxter_interface.Gripper('left').calibrate()
        baxter_interface.Gripper('right').calibrate()
        rospy.loginfo("Both grippers calibrated")


    def setupGrippers(self):
        """Set up Baxter grippers"""
        gripper_names = ['right', 'left']
        grippers = {}

        for gripper_name in gripper_names:
            gripper_control = baxter_interface.Gripper(gripper_name)

            grippers[gripper_name] = gripper_control
        self.grippers = grippers


    def resetGrippers(self):
        """Reset grippers so that they are not holding anything"""
        self.grippers['right'].open()
        self.grippers['left'].open()
        rospy.sleep(0.5)
        self.grippers['right'].close()
        self.grippers['left'].close()
        rospy.sleep(0.5)


    def setGrip(gripper_name, dist):
        """
        Set gripper width to desired distance (in centimetres)

        gripper_name: string name of desired gripper (left/right)
        dist: desired grasp width (in cm)
        """
        self.grippers[gripper_name].command_position(cm2grip(dist)) 


    # ------------------------------ Reset Pose Action ------------------------------

    def setupArmPoses(self):
        """Set up arm poses for poking actions"""
        rospy.loginfo("Arm set up: Planning reset joint values")

        if self.planBothArmReset():
            rospy.loginfo("Arm set up: Executing reset joint values")
            self.controls['both_arms'].go(wait=True)
            self.controls['both_arms'].stop()


    def planRightArmUp(self):
        """
        Plan reset from post-poke action to pre-poke joint values without disturbing 
        colliding with objects by moving right arm upwards 10 centimeters

        return: upward 10cm right arm motion plan if successful, else None
        """
        waypoints = []
        curr_pose = self.controls['right_arm'].get_current_pose().pose

        p1_pose = copy.deepcopy(curr_pose)
        p1_pose.position.z = POKE_HEIGHT + 0.10
        waypoints.append(p1_pose)

        # originally used for resetting by pose, we do this by joint values 
        # in planBothArmReset 
        # p2_pose = copy.deepcopy(p1_pose)
        # p2_pose.position.x = RIGHT_ARM_REST_POSE.position.x
        # p2_pose.position.y = RIGHT_ARM_REST_POSE.position.y
        # p2_pose.orientation = RIGHT_ARM_REST_POSE.orientation
        # waypoints.append(p2_pose)

        rospy.loginfo("Resetting: Planning upward move")
        plan = multiTryCartesianPlan(self.controls['right_arm'], waypoints)

        if plan is None:
            rospy.loginfo("Resetting: [ERROR] No upward move plan")
        else:
            rospy.loginfo("Resetting: Upward move plan found")
            plan = slowDownPlan(plan, 0.4)
        return plan


    def planBothArmReset(self):
        """
        Plan rest joint values for both arms.

        return: True, if successful plan found, else False 
        """
        self.controls['both_arms'].set_joint_value_target(REST_JOINT_VALUES) 

        rospy.loginfo("Resetting: Planning reset joint values")
        plan = multiTryPlan(self.controls['both_arms']) # is this actually doing something? verify

        if plan is None:
            rospy.loginfo("Resetting: [ERROR] No reset joint value plan")
            return False
        else:
            rospy.loginfo("Resetting: Reset joint values plan found")
            return True


    def doReset(self):
        """
        Plan and execute post-poke reset to pre-poke pose

        return: [bool1, bool2]; bool1 is True if successful upward motion plan 
                is found, else False; bool2 is True if joint value rest plan is 
                found, else False 
        """
        upwardPlan = self.planRightArmUp()
        if upwardPlan is not None:
            rospy.loginfo("Resetting: Executing upward move")
            self.controls['right_arm'].execute(upwardPlan, wait=True)
            self.controls['right_arm'].stop()
        else:
            return [False, False]

        if self.planBothArmReset():
            rospy.loginfo("Resetting: Executing reset joint values")
            self.controls['both_arms'].go(wait=True)
            self.controls['both_arms'].stop()
        else:
            return [True, False]

        self.grippers['right'].close()

        return [True, True]


    # ------------------------------ Poke Action ------------------------------

    def planPoke(self, x, y, theta, length):
        """
        Use MoveIt to make a poke plan using a given arm and poke specifications
        controls: end-effector commanders

        x: poke centre x-axis value (in metres)
        y: poke centre y-axis value (in metres)
        theta: angle of poke (in radians) w.r.t to baxter base frame
        length: length of poke (in metres)
        return: poke plan if successful, else None
        """
        xm = utils.norm2width(x)
        ym = utils.norm2height(y)
        tm = norm2wrist(theta)
        lm = prop2len(length)

        waypoints = []
        curr_pose = self.controls['right_arm'].get_current_pose().pose

        p1_pose = copy.deepcopy(curr_pose)
        p1_pose.position.x = xm + np.cos(tm) * lm
        p1_pose.position.y = ym + np.sin(tm) * lm
        p1_pose.orientation = geometry_msgs.msg.Quaternion( *tf.transformations.quaternion_from_euler(-np.pi, 0, reorientYaw( tm)))
        waypoints.append(p1_pose)

        p2_pose = copy.deepcopy(p1_pose)
        p2_pose.position.z = POKE_HEIGHT
        waypoints.append(p2_pose)

        p3_pose = copy.deepcopy(p2_pose)
        p3_pose.position.x = xm + np.cos(tm + np.pi) * lm
        p3_pose.position.y = ym + np.sin(tm + np.pi) * lm
        waypoints.append(p3_pose)

        pokeStr = formatPoke(x,y,theta,length)
        rospy.loginfo("Poking: Planning poke: " + pokeStr)

        plan = multiTryCartesianPlan(self.controls['right_arm'], waypoints, max_tries=1)
        if plan is None:
            rospy.loginfo("Poking: [ERROR] No plan")
        else:
            rospy.loginfo("Poking: Plan found")
            plan = slowDownPlan(plan, 0.4)
        return plan


    def executePoke(self, plan):
        """
        Execute a poke plan with given arm and gripper
        
        plan: poke plan for right arm
        """
        rospy.loginfo("Poking: Executing poke")
        self.grippers['right'].close()
        rospy.sleep(0.1)
        self.controls['right_arm'].execute(plan, wait=True)


    def doPoke(self, x, y, theta, length):
        """
        Plan and execute a poke action

        x: poke centre x-axis value (in metres)
        y: poke centre y-axis value (in metres)
        theta: angle of poke (in radians) w.r.t to baxter base frame
        length: length of poke (in metres)
        return: True if successful poke plan is found, else False       
        """
        pokePlan = self.planPoke(x, y, theta, length)
        if pokePlan:
            self.executePoke(pokePlan)
            return True
        return False