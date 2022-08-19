#!/usr/bin/python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import argparse

import rospy
try:
    import xacro_jade as xacro
except ImportError:
    import xacro


from intera_core_msgs.msg import (
    URDFConfiguration,
)

def xacro_parse(filename):
    doc = xacro.parse(None, filename)
    xacro.process_doc(doc, in_order=True)
    return doc.toprettyxml(indent='  ')

def send_urdf(parent_link, root_joint, urdf_filename, duration):
    """
    Send the URDF Fragment located at the specified path.

    @param parent_link: parent link to attach the URDF fragment to
                        (usually <side>_hand)
    @param root_joint: root link of the URDF fragment (usually <side>_gripper_base)
    @param urdf_filename: path to the urdf XML file to load into xacro and send
    @param duration: duration to repeat sending the URDF to ensure it is received
    """
    msg = URDFConfiguration()
    # The updating the time parameter tells
    # the robot that this is a new configuration.
    # Only update the time when an updated internal
    # model is required. Do not continuously update
    # the time parameter.
    msg.time = rospy.Time.now()
    # link to attach this urdf to onboard the robot
    msg.link = parent_link
    # root linkage in your URDF Fragment
    msg.joint = root_joint
    msg.urdf = xacro_parse(urdf_filename)
    pub = rospy.Publisher('/robot/urdf', URDFConfiguration, queue_size=10)
    rate = rospy.Rate(5) # 5hz
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        if (rospy.Time.now() - msg.time) > rospy.Duration(duration):
            break

def main():
    """RSDK URDF Fragment Example:
    This example shows a proof of concept for
    adding your URDF fragment to the robot's
    onboard URDF (which is currently in use).
    """
    rospy.init_node('rsdk_configure_urdf', anonymous=True)

    link = rospy.get_param('~link')
    joint = rospy.get_param('~joint')
    file = rospy.get_param('~file')
    duration = rospy.get_param('~duration')

    send_urdf(link, joint, file, duration)
    return 0

if __name__ == '__main__':
    sys.exit(main())
