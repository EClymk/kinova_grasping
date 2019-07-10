#! /usr/bin/env python

import rospy
import tf.transformations as tft

import numpy as np

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg
import sensor_msgs.msg
import actionlib_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError


from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import position_client, move_to_position
from helpers.joints_action_client import joint_angle_client
from helpers.covariance import generate_cartesian_covariance

import os
import time
import random
import pickle
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

import inspect


currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
grandgrandparentdir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(currentdir))))

VELOCITY_CONTROL = 1            # when 0, position control; when 1, velocity control
DATA_LENGTH = 50              # set the total data length
POSE_FREQ = 10                   # the frequency of input
ROLLOUT_AMOUNT = 5         # set the number of rollout sets
K = 0.02                   # coefficient of input to motion
target_position = (0, -0.5, 0.4)
bridge = CvBridge()
MOVING = True               # when false, this program is to be shut down

class rollout_data:
    image = []
    torque = [0,0,0,0,0,0,0]
    pose = [0,0,0]
    orientation = [0,0,0]
    joint_angle = [0,0,0,0,0,0,0]
    joint_velocity = [0,0,0,0,0,0,0]
    action = []
    cmd = []
    reward = []


def jointangle_callback(data):
    global temp_angles
    temp_angles = data


def return_home_angle():
    home_srv = rospy.ServiceProxy('/j2s7s300_driver/in/home_arm', kinova_msgs.srv.HomeArm)
    home_srv()
    time.sleep(0.2)
    ini_angles = temp_angles
    time.sleep(0.2)
    big_angle_err = True
    while big_angle_err:
        while abs(temp_angles.joint1 - 283) > 10:
            if temp_angles.joint1 > 283:
                CURRENT_VELOCITY = [-30, 0, 0, 0, 0, 0, 0]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
            if temp_angles.joint1 < 283:
                CURRENT_VELOCITY = [30, 0, 0, 0, 0, 0, 0]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
        CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0, 0]
        time.sleep(0.1)

        while abs(temp_angles.joint3 - 0) > 10:
            if temp_angles.joint3 > 0:
                CURRENT_VELOCITY = [0, 0, -30, 0, 0, 0, 0]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
            if temp_angles.joint3 < 0:
                CURRENT_VELOCITY = [0, 0, 30, 0, 0, 0, 0]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
        CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0, 0]
        time.sleep(0.1)

        while abs(temp_angles.joint4 - 90) > 10:
            if temp_angles.joint4 > 90:
                CURRENT_VELOCITY = [0, 0, 0, -30, 0, 0, 0]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
            if temp_angles.joint4 < 90:
                CURRENT_VELOCITY = [0, 0, 0, 30, 0, 0, 0]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
        CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0, 0]
        time.sleep(0.1)

        while abs(temp_angles.joint5 - 265) > 10:
            if temp_angles.joint5 > 265:
                CURRENT_VELOCITY = [0, 0, 0, 0, -60, 0, 0]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
            if temp_angles.joint5 < 265:
                CURRENT_VELOCITY = [0, 0, 0, 0, 60, 0, 0]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
        CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0, 0]
        time.sleep(0.1)

        while abs(temp_angles.joint7 - 287) > 10:
            if temp_angles.joint7 > 287:
                CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0, -60]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
            if temp_angles.joint7 < 287:
                CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0, 60]
                velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
                r.sleep()
        CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0, 0]
        time.sleep(0.1)

        big_angle_err = False

    home_srv()
    time.sleep(0.1)


if __name__ == '__main__':

    rospy.init_node('kinova_angle_home')
    rospy.Subscriber('/j2s7s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, jointangle_callback, queue_size=1)

    velo_pub = rospy.Publisher('/j2s7s300_driver/in/joint_velocity', kinova_msgs.msg.JointVelocity, queue_size=1)
    CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0, 0]
    velo_pub.publish(kinova_msgs.msg.JointVelocity(*CURRENT_VELOCITY))
    r = rospy.Rate(100)

    from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform

    start_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    stop_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/stop_force_control', kinova_msgs.srv.Stop)

    return_home_angle()

    # data = dataIO.read_pickle()
    # img = cv2.imread(str(data[0][0]))
    # cv2.imshow('1', img)
    # time.sleep(10)
