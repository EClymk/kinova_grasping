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


def policy(stat):
    act = [0.1,0.1, 0.1]
    print('---')
    print(act)
    return act

bridge = CvBridge()

class AgentROS():
    def __init__(self):
        rospy.init_node('agent_ros_node')
        self._init_pubs_and_subs()
        r = rospy.Rate(1)
        r.sleep()
        bridge = CvBridge()

    def _init_pubs_and_subs(self):
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback, queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/joint_torques', kinova_msgs.msg.JointTorque, self.torque_callback,
                         queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/joint_state', sensor_msgs.msg.JointState, self.joint_callback, queue_size=1)
        # rospy.Subscriber('/j2s7s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, self.jointangle_callback,
        #                  queue_size=1)
        # if VELOCITY_CONTROL == 1:
        self.cmd_pub = rospy.Publisher('/agent_ros/position_feed', msgs.msg.ActionCommand, queue_size=1)            # [x y z home]
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

    rollout_observation_image = []
    rollout_observation_torque = []
    rollout_observation_pose = []
    rollout_observation_orientation = []
    rollout_observation_joint_angle = []
    rollout_observation_joint_velocity = []
    rollout_observation_action = []
    # rollout_observation_cmd = []
    rollout_observation_reward = []
    target_position = (0, -0.5, 0.4)
    stat = []
    rollout_temp = RollOutData()


    def color_callback(self, color_data):
        original_image = bridge.imgmsg_to_cv2(color_data, 'bgr8')

        # Crop a square out of the middle of the depth and resize it to 300*300
        crop_size = 480
        self.rollout_temp.image = cv2.resize(original_image[(480 - crop_size) // 2:(480 - crop_size) // 2 + crop_size,
                                        (640 - crop_size) // 2:(640 - crop_size) // 2 + crop_size], (480, 480))

    # def jointangle_callback(self, data):
    #     global temp_angles
    #     temp_angles = data
    #     print('!!')

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

    def timer_callback(self):
        self.rollout_observation_image.append(self.rollout_temp.image)
        self.rollout_observation_torque.append(self.rollout_temp.torque)
        self.rollout_observation_pose.append(self.rollout_temp.pose)
        self.rollout_observation_orientation.append(self.rollout_temp.orientation)
        self.rollout_observation_joint_angle.append(self.rollout_temp.joint_angle)
        self.rollout_observation_joint_velocity.append(self.rollout_temp.joint_velocity)
        self.rollout_observation_action.append(self.rollout_temp.action)
        # self.rollout_observation_cmd.append(self.rollout_temp.cmd)
        self.rollout_observation_reward.append(self.reward(self.target_position, self.rollout_temp.pose))
        self.stat = 1

    def rollout_clean(self):
        self.rollout_observation_image = []
        self.rollout_observation_torque = []
        self.rollout_observation_pose = []
        self.rollout_observation_orientation = []
        self.rollout_observation_joint_angle = []
        self.rollout_observation_joint_velocity = []
        self.rollout_observation_action = []
        # self.rollout_observation_cmd = []
        self.rollout_observation_reward = []

    def reward(self, target_position, current_position):
        pose_data = -np.square(np.array(target_position) - np.array(current_position))
        return pose_data

    def home_client(self):
        rospy.wait_for_service('/agent_ros/srv/home')
        try:
            home_req = rospy.ServiceProxy('/agent_ros/srv/home', msgs.srv.Home)
            home1 = home_req(1)
            return home1.done
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def rollout(self, length, amount):
        self.rollout_clean()
        r = rospy.Rate(10)
        for i in range(amount):

            # rospy.Timer(rospy.Duration(0.1), self.timer_callback)
            for j in range(length):
                self.rollout_temp.action = policy(self.stat)
                print(self.rollout_temp.action)
                self.cmd_pub.publish(msgs.msg.ActionCommand(*self.rollout_temp.action))
                self.timer_callback()
                r.sleep()


            act = self.rollout_temp.action
            print(act)
            self.home_client()
            # rospy.Timer.shutdown()
            time.sleep(1)
        print(self.rollout_observation_action)
        print(len(self.rollout_observation_image))
        rollout_observation = [
            np.array(self.rollout_observation_action).reshape((amount, length, 3)),
            # np.array(self.rollout_observation_cmd).reshape((amount, length, 6)),
            np.array(self.rollout_observation_reward).reshape((amount, length, 3)),
            np.array(self.rollout_observation_torque).reshape((amount, length, 7)),
            np.array(self.rollout_observation_pose).reshape((amount, length, 3)),
            np.array(self.rollout_observation_orientation).reshape((amount, length, 4)),
            np.array(self.rollout_observation_joint_angle).reshape((amount, length, 10)),
            np.array(self.rollout_observation_joint_velocity).reshape((amount, length, 10)),
            np.array(self.rollout_observation_image).reshape((amount, length, 691200))
        ]
        return rollout_observation

agent = AgentROS()
while True:
    agent.rollout(20,3)