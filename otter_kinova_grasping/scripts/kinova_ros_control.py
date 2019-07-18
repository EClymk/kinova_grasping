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
import msgs.msg
import msgs.srv


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


class AgentKinova():
    VELOCITY_CONTROL = 1  # when 0, position control; when 1, velocity control
    POSE_FREQ = 10  # the frequency of input
    target_position = (0, -0.5, 0.4)
    bridge = CvBridge()
    MOVING = True  # when false, this program is to be shut down
    x_v = 0
    y_v = 0
    z_v = 0
    temp_angles = []
    home_angles = []
    pose = []
    home_xyz = [0.09, -0.446, 0.375]
    home_orientation = [0.708, -0.019, 0.037, 0.705]
    limit_xyz = [-0.1, 0.2, -0.7, -0.4, 0.365, 0.465]
    r = []

    def __init__(self):
        rospy.init_node('kinova_agent_node')
        self._init_pubs_and_subs()
        r = rospy.Rate(100)
        r.sleep()
        self.move_home_init()
        r.sleep()
        self.home_angles = self.temp_angles
        print('init finished')

    def _init_pubs_and_subs(self):
        # rospy.Subscriber('/j2s7s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, self.robot_wrench_callback,
        #                  queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, self.pose_callback, queue_size=1)
        # rospy.Subscriber('/j2s7s300_driver/out/joint_state', sensor_msgs.msg.JointState, self.joint_callback, queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, self.joint_angle_callback,
                          queue_size=1)
        # rospy.Subscriber('/agent_ros/position_feed', Float32MultiArray, self.move_callback_velocity_control, queue_size=1)
        # if VELOCITY_CONTROL == 1:
        rospy.Subscriber('/agent_ros/position_feed', msgs.msg.ActionCommand, self.move_callback_velocity_control, queue_size=1)
        # else:
        #     rospy.Subscriber('/target_goal', Float32MultiArray, move_callback_position_control, queue_size=1)
        self.velo_pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)
        CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0]
        self.velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
        self.r = rospy.Rate(100)
        self.home_srv = rospy.Service('/agent_ros/srv/home', msgs.srv.Home, self.move_home)
        self.home_and_limit_srv = rospy.Service('/agent_ros/srv/home_and_limit_range', msgs.srv.HomeAndLimit,
                                                self.home_and_limit)

    def home_and_limit(self, req):
        if req.home_xyz:
            self.home_xyz = req.home_xyz
            self.home_orientation = req.home_orientation
            self.limit_xyz = req.limit_range
            time.sleep(0.5)
            # self.MOVING = True
        return 1

    def pose_callback(self, pose_data):
        self.pose = [pose_data.pose.position.x,
                             pose_data.pose.position.y,
                             pose_data.pose.position.z
                             ]

    def joint_angle_callback(self, data):
        self.temp_angles = data

    def robot_wrench_callback(self, msg):
        if abs(msg.wrench.force.x) > 5.0 or abs(msg.wrench.force.y) > 5.0 or abs(msg.wrench.force.z) > 7.0:
            rospy.logerr('Force Detected. Stopping.')
            self.MOVING = False
            print(msg.wrench)
            print(self.temp_angles)
            if self.temp_angles:
                joint_angle_client([self.temp_angles.joint1, self.temp_angles.joint2, self.temp_angles.joint3,
                                    self.temp_angles.joint4, self.temp_angles.joint5, self.temp_angles.joint6,
                                    self.temp_angles.joint7])
                rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)

    def move_home_init(self):
        # move_to_position([x_r,y_r,z_r], [0.072, 0.6902, -0.7172, 0.064])
        self.MOVING = False
        move_to_position(self.home_xyz, self.home_orientation)
        time.sleep(0.5)
        self.home_angles = self.temp_angles
        self.MOVING = True
        return 1

    def move_home(self, req):
        if req.home:
            self.MOVING = False
            self.x_v = 0
            self.y_v = 0
            self.z_v = 0
            joint_angle_client([self.home_angles.joint1, self.home_angles.joint2, self.home_angles.joint3,
                                self.home_angles.joint4, self.home_angles.joint5, self.home_angles.joint6,
                                self.home_angles.joint7])
            time.sleep(0.5)
            print('home!')
            self.MOVING = True
        return 1

    def move_callback_velocity_control(self, data):
        # disp = [data.data[0], data.data[1], data.data[2]]
        x_v = data.x * self.POSE_FREQ
        y_v = data.y * self.POSE_FREQ
        z_v = data.z * self.POSE_FREQ
        # self.rollout_temp.action = data.data
        # self.rollout_temp.cmd = [x_v, y_v, z_v, 0, 0, 0]
        if self.pose[0] > self.limit_xyz[1]:
            x_v = -abs(x_v)
        elif self.pose[0] < self.limit_xyz[0]:
            x_v = abs(x_v)
        if self.pose[1] > self.limit_xyz[3]:
            y_v = -abs(y_v)
        elif self.pose[1] < self.limit_xyz[2]:
            y_v = abs(y_v)
        if self.pose[2] > self.limit_xyz[5]:
            z_v = -abs(z_v)
        elif self.pose[2] < self.limit_xyz[4]:
            z_v = abs(z_v)
        self.x_v = x_v
        self.y_v = y_v
        self.z_v = z_v
        # move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])

    def __main__(self):
        while not rospy.is_shutdown():
            if self.MOVING:
                CURRENT_VELOCITY = [self.x_v, self.y_v, self.z_v, 0, 0, 0]
                self.velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
                self.r.sleep()


if __name__ == '__main__':


    # rospy.Timer(rospy.Duration(0.1),timer_callback)
    # from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
    #
    # start_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    # stop_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/stop_force_control', kinova_msgs.srv.Stop)
    # time.sleep(1)
    # stop_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start', kinova_msgs.srv.Start)
    # home_srv = rospy.ServiceProxy('/j2s7s300_driver/in/home_arm', kinova_msgs.srv.HomeArm)
    # home_srv()
    # time.sleep(1)

    kinova = AgentKinova()
    kinova.__main__()
    # data = dataIO.read_pickle()
    # img = cv2.imread(str(data[0][0]))
    # cv2.imshow('1', img)
    # time.sleep(10)
