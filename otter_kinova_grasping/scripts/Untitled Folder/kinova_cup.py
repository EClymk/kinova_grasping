#! /usr/bin/env python

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


from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import position_client, move_to_position
from helpers.joints_action_client import joint_angle_client
from helpers.covariance import generate_cartesian_covariance
import kinova_angle_home

import os
import time
import random
import pickle
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

import inspect
import collections

from .kinova_agnet_base import AgentROSbase


class CupAgentROS(AgentROSbase):
    def __init__(self):
        AgentROSbase.__init__()

    def reward(self, obs, action):
        return 0.1

    def policy(self, state):
        return [0, 0, 0]
