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


currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
grandgrandparentdir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(currentdir))))

IMAGE_WIDTH = 128
STATE_DIM = 3*IMAGE_WIDTH*IMAGE_WIDTH
CROP_SIZE = 360
KINOVA_HOME_ANGLE = [4.543, 3.370, -0.264, 0.580, 2.705, 4.350, 6.425, 0, 0,0 ]
KINOVA_HOME_XYZ = [0.09, -0.446, 0.375]
KINOVA_HOME_ORIENTATION = [0.708, -0.019, 0.037, 0.705]
KINOVA_LIMIT = [-0.1, 0.2, -0.7, -0.4, 0.365, 0.465]
K = 0.02        # velocity parameter v = K*input(from -1 to 1)m/s

def policy1(stat):
    act = [1,1, 1]
    act =np.array( [np.random.uniform(-1,1) , np.random.uniform(-1,1) , np.random.uniform(-1,1) ])
    print('---')
    print(act)
    return act

bridge = CvBridge()

class AgentROSbase(metaclass=abc.ABCMeta):
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
        done = []

    def _init_home_and_limit(self):
        rospy.wait_for_service('/agent_ros/srv/home_and_limit_range')
        try:
            home_limit_req = rospy.ServiceProxy('/agent_ros/srv/home_and_limit_range', msgs.srv.HomeAndLimit)
            home2 = home_limit_req(KINOVA_HOME_XYZ, KINOVA_HOME_ORIENTATION, KINOVA_LIMIT)
            return home2.done
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def home_client(self):
        rospy.wait_for_service('/agent_ros/srv/home')
        try:
            home_req = rospy.ServiceProxy('/agent_ros/srv/home', msgs.srv.Home)
            home1 = home_req(1)
            return home1.done
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    rollout_observation_image = []
    rollout_observation_torque = []
    rollout_observation_pose = []
    rollout_observation_orientation = []
    rollout_observation_joint_angle = []
    rollout_observation_joint_velocity = []
    rollout_action = []
    rollout_reward = []
    rollout_done = []
    # rollout_observation_cmd = []
    # target_position = (0, -0.5, 0.4)
    stat = []
    rollout_temp = RollOutData()


    def color_callback(self, color_data):
        original_image = bridge.imgmsg_to_cv2(color_data, 'rgb8')

        # Crop a square out of the middle of the depth and resize it to 300*300
        # self.rollout_temp.image = cv2.resize(original_image[(480 - crop_size) // 2:(480 - crop_size) // 2 + crop_size,
        #                                 (640 - crop_size) // 2:(640 - crop_size) // 2 + crop_size], (IMAGE_WIDTH, IMAGE_WIDTH))
        self.rollout_temp.image = cv2.resize(original_image[0:CROP_SIZE,
                                        (640 - CROP_SIZE) // 2:(640 - CROP_SIZE) // 2 + CROP_SIZE], (IMAGE_WIDTH, IMAGE_WIDTH))/255
        # print(type(self.rollout_temp.image))

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

    def save_rollout(self):
        self.rollout_observation_image.append(self.rollout_temp.image)
        self.rollout_observation_torque.append(self.rollout_temp.torque)
        self.rollout_observation_pose.append(self.rollout_temp.pose)
        self.rollout_observation_orientation.append(self.rollout_temp.orientation)
        self.rollout_observation_joint_angle.append(self.rollout_temp.joint_angle)
        self.rollout_observation_joint_velocity.append(self.rollout_temp.joint_velocity)
        self.rollout_action.append(self.rollout_temp.action)
        # self.rollout_observation_cmd.append(self.rollout_temp.cmd)
        self.rollout_reward.append(self.reward(self.target_position, self.rollout_temp.pose))
        self.rollout_done.appned(self.rollout_temp.done)

    def rollout_clean(self):
        self.rollout_observation_image = []
        self.rollout_observation_torque = []
        self.rollout_observation_pose = []
        self.rollout_observation_orientation = []
        self.rollout_observation_joint_angle = []
        self.rollout_observation_joint_velocity = []
        self.rollout_action = []
        # self.rollout_observation_cmd = []
        self.rollout_reward = []
        self.rollout_done = []

    def reward(self, obs, action):
        pass
        # pose_data = -np.square(np.array(target_position) - np.array(current_position))
        # reward = np.sum(pose_data)
        # print(reward)
        # return reward



    def reset(self):
        self.home_client()
        time.sleep(1)
        return self.observe()

    def observe_state(self):
        return self.rollout_temp.image.flatten()

    def get_state_dim(self):
        return STATE_DIM

    def get_action_dim(self):
        return 3

    def step(self,actions):
        pub_action = [K*i for i in actions]
        self.cmd_pub.publish(msgs.msg.ActionCommand(*pub_action))
        return self.observe_state(), self.reward(self.target_position, self.rollout_temp.pose),False

    def policy(self, state):
        pass

    def rollout(self, num_horizon,policy, init_std=1):
        # if policy is None:
        #     def policy(_, t, noise=None):
        #         return np.random.normal(size=self.get_action_dim(), scale=init_std)
        r = rospy.Rate(10)
        states, actions, costs, info = (
            np.zeros([num_horizon] + [self.get_state_dim()]),
            np.zeros([num_horizon] + [self.get_action_dim()]),
            np.zeros([num_horizon]),
            [])
        infos = collections.defaultdict(list)
        current_state = self.reset()
        # print(current_state)
        for t in range(num_horizon):
            states[t] = current_state
            actions[t] = policy(states[t])
            current_state, costs[t], done = self.step(actions[t])
            r.sleep()

            # for k, v in info.items():
            #     infos[k].append(v)
        # if self.currently_logging:
        #     log_entry = collections.OrderedDict()
        #     log_entry['episode_number'] = self.episode_number
        #     log_entry['mean_cost'] = costs.mean()
        #     log_entry['total_cost'] = costs.sum()
        #     log_entry['final_cost'] = costs[-1]
        #     for k, v in infos.items():
        #         v = np.array(v)
        #         log_entry['mean_%s' % k] = v.mean()
        #         log_entry['total_%s' % k] = v.sum()
        #         log_entry['final_%s' % k] = v[-1]
        #     self.log_entry(log_entry)
        #     self.episode_number += 1
        return states, actions, costs

    def rollouts(self, num_rollouts, num_horizon, policy=None):
        if policy is None:
            policy = self.policy()
        states, actions, costs = (
            np.empty([num_rollouts, num_horizon] + [self.get_state_dim()]),
            np.empty([num_rollouts, num_horizon] + [self.get_action_dim()]),
            np.empty([num_rollouts, num_horizon])
        )
        # infos = [None] * num_rollouts
        # rollouts = tqdm.trange(num_rollouts, desc='Rollouts') if show_progress else range(num_rollouts)
        for i in range(num_rollouts):
            states[i], actions[i], costs[i] = \
            self.rollout(num_horizon, policy)
        self.reset()
        return [states, actions, costs]

    # def rollout(self, length, amount):
    #     self.rollout_clean()
    #     r = rospy.Rate(10)
    #     for i in range(amount):
    #         current_state = self.reset()
    #         # rospy.Timer(rospy.Duration(0.1), self.timer_callback)
    #         for j in range(length):
    #             self.stat = current_state
    #             self.rollout_temp.action = policy(self.stat)
    #             print(self.rollout_temp.action)
    #             self.cmd_pub.publish(msgs.msg.ActionCommand(*self.rollout_temp.action))
    #             self.rollout_temp.done = self.isdone()
    #             self.save_rollout()
    #             r.sleep()
    #             current_state = self.observe()
    #         # act = self.rollout_temp.action
    #         # print(act)
    #         # rospy.Timer.shutdown()
    #     self.home_client()
    #     rollout_observation = [
    #         np.array(self.rollout_observation_action).reshape((amount, length, 3)),
    #         # np.array(self.rollout_observation_cmd).reshape((amount, length, 6)),
    #         np.array(self.rollout_observation_reward).reshape((amount, length, 3)),
    #         np.array(self.rollout_observation_torque).reshape((amount, length, 7)),
    #         np.array(self.rollout_observation_pose).reshape((amount, length, 3)),
    #         np.array(self.rollout_observation_orientation).reshape((amount, length, 4)),
    #         np.array(self.rollout_observation_joint_angle).reshape((amount, length, 10)),
    #         np.array(self.rollout_observation_joint_velocity).reshape((amount, length, 10)),
    #         np.array(self.rollout_observation_image).reshape((amount, length, 691200))
    #     ]
    #     return rollout_observation

class IO(object):
    def __init__(self, file):
        self.file = file
    def to_pickle(self, data):
        with open(self.file,'wb') as f:
            pickle.dump(data,f)
    def read_pickle(self):
        with open(self.file, 'rb') as f:
            data = pickle.load(f)
        return data


agent = AgentROSbase()
DIR1 = grandgrandparentdir
DIR2 = str(rospy.get_time())
DIR = DIR1+'/data/'+DIR2
os.makedirs(DIR, mode=0o777)
dataIO = IO(DIR + '/data.pkl')


num_rollouts = 5
horizon = 50

t1 = time.time()
rollout = agent.rollouts(num_rollouts,horizon)
t2 = time.time()

print(' time : ', t2- t1)
dataIO.to_pickle(rollout)
# agent.rollouts(3,20)

time.sleep(20)