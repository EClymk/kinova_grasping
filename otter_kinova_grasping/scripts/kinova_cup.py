#! /usr/bin/env python

import os
import inspect
# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# #print ("current_dir=" + currentdir)
# os.sys.path.insert(0,currentdir)
import rospy
import tf.transformations as tft

import numpy as np
import abc

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg
import sensor_msgs.msg
import actionlib_msgs.msg
# import otter_kinova_grasping.ActionCommand.msg
import msgs.msg
import msgs.srv
import cv2
from cv_bridge import CvBridge, CvBridgeError
from kinova_agent_base import *

# from otter_kinova_grasping.otter_kinova_grasping.scripts.helpers.gripper_action_client import set_finger_positions
# from otter_kinova_grasping.otter_kinova_grasping.scripts.helpers.position_action_client import position_client, move_to_position
# from otter_kinova_grasping.otter_kinova_grasping.scripts.helpers.joints_action_client import joint_angle_client
# from otter_kinova_grasping.otter_kinova_grasping.scripts.helpers.covariance import generate_cartesian_covariance
# import kinova_angle_home


import time
import random
import pickle
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


import collections


class CupAgentROS(AgentROSbase):
    def __init__(self):
        # rospy.init_node('agent_ros_node')
        AgentROSbase.__init__(self)

    def reward(self, obs, action):
        return 0.1

    def policy(self, state):
        return [0, -0.5, 0]

cupAgent = CupAgentROS()
print('set')
cupAgent.rollouts(3, 30, cupAgent.policy)
