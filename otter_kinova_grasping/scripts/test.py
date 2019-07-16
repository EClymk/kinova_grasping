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
import kinova_angle_home

import os
import time
import random
import pickle
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

import inspect


class AgentROS():
    def __init(self):
        rospy.init_node('agent_ros_node')
        self._init_pubs_and_subs(self)
        r = rospy.rate(1)
        r.sleep()

    def _init_pubs_and_subs(self):
        rospy.Subscriber('/j2s7s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, self.robot_wrench_callback,
                         queue_size=1)
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback, queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/joint_torques', kinova_msgs.msg.JointTorque, self.torque_callback,
                         queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/joint_state', sensor_msgs.msg.JointState, self.joint_callback, queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, self.jointangle_callback,
                         queue_size=1)
        # if VELOCITY_CONTROL == 1:
        rospy.Subscriber('/target_goal', Float32MultiArray, self.move_callback_velocity_control, queue_size=1)
        cmd_pub = rospy.Publisher('/agent_ros/position_feed', Float32MultiArray, queue_size=1)
        # else:
        #     rospy.Subscriber('/target_goal', Float32MultiArray, move_callback_position_control, queue_size=1)

    class RollOutData:
        image = []
        torque = [0, 0, 0, 0, 0, 0, 0]
        pose = [0, 0, 0]
        orientation = [0, 0, 0]
        joint_angle = [0, 0, 0, 0, 0, 0, 0]
        joint_velocity = [0, 0, 0, 0, 0, 0, 0]
        action = []
        cmd = []
        reward = []


    def move_callback_velocity_control(self, data):
        # disp = [data.data[0], data.data[1], data.data[2]]
        global x_v
        global y_v
        global z_v
        x_v = data.data[0] * POSE_FREQ * K
        y_v = data.data[1] * POSE_FREQ * K
        z_v = data.data[2] * POSE_FREQ * K
        self.rollout_temp.action = data.data
        self.rollout_temp.cmd = [x_v, y_v, z_v, 0, 0, 0]
        if self.rollout_temp.pose[0] > 0.2:
            x_v = -abs(x_v)
        elif self.rollout_temp.pose[0] < -0.1:
            x_v = abs(x_v)
        if self.rollout_temp.pose[1] > -0.4:
            y_v = -abs(y_v)
        elif self.rollout_temp.pose[1] < -0.7:
            y_v = abs(y_v)
        if self.rollout_temp.pose[2] > 0.465:
            z_v = -abs(z_v)
        elif self.rollout_temp.pose[2] < 0.365:
            z_v = abs(z_v)
        return x_v, y_v, z_v
        # move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])

    def move_callback_position_control(self, data):
        disp = self.rollout_temp.pose
        for i in len(self.rollout_temp.pose):
            self.rollout_temp.pose[i] += data[i] * K
        # move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])          # now it's a specified orientation
        move_to_position(disp, [0.708, -0.019, 0.037, 0.705])  # now it's a specified orientation

    def color_callback(self, color_data):

        original_image = bridge.imgmsg_to_cv2(color_data, 'bgr8')

        # Crop a square out of the middle of the depth and resize it to 300*300
        crop_size = 480
        self.rollout_temp.image = cv2.resize(original_image[(480 - crop_size) // 2:(480 - crop_size) // 2 + crop_size,
                                        (640 - crop_size) // 2:(640 - crop_size) // 2 + crop_size], (480, 480))

    def jointangle_callback(self, data):
        global temp_angles
        temp_angles = data
        print('!!')

    def torque_callback(self, torque_data):
        self.rollout_temp.torque = [torque_data.joint1,
                               torque_data.joint2,
                               torque_data.joint3,
                               torque_data.joint4,
                               torque_data.joint5,
                               torque_data.joint6,
                               torque_data.joint7
                               ]

    def pose_callback(self, pose_data):
        self.rollout_temp.pose = [pose_data.pose.position.x,
                             pose_data.pose.position.y,
                             pose_data.pose.position.z
                             ]

        self.rollout_temp.orientation = [pose_data.pose.orientation.x,
                                    pose_data.pose.orientation.y,
                                    pose_data.pose.orientation.z,
                                    pose_data.pose.orientation.w
                                    ]

    def joint_callback(self, joint_data):
        self.rollout_temp.joint_angle = list(joint_data.position)
        self.rollout_temp.joint_velocity = list(joint_data.velocity)


    def reward(self, target_position, current_position):
        pose_data = -np.square(np.array(target_position) - np.array(current_position))
        return pose_data

