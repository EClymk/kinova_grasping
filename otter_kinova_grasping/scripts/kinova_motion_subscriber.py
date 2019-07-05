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
from helpers.covariance import generate_cartesian_covariance

import os
import time
import random
import pickle
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


VELOCITY_CONTROL = 1            # when 0, position control; when 1, velocity control
DATA_LENGTH = 50               # set the total data length
POSE_FREQ = 1                   # the frequency of input

bridge = CvBridge()

x_v = 0
y_v = 0
z_v = 0


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


class rollout_data:
    image = []
    torque = []
    pose = []
    orientation = []
    joint_angle = []
    joint_velocity = []


def move_home():
    global x_r
    global y_r
    global z_r
    x_r = -0.03
    y_r = -0.538
    z_r = 0.375
    move_to_position([x_r,y_r,z_r], [0.072, 0.6902, -0.7172, 0.064])
    time.sleep(0)


def move_callback_velocity_control(data):
    # disp = [data.data[0], data.data[1], data.data[2]]
    global x_v
    global y_v
    global z_v
    x_v = (data.data[0] - rollout_temp.pose[0])*POSE_FREQ
    y_v = (data.data[1] - rollout_temp.pose[1])*POSE_FREQ
    z_v = (data.data[2] - rollout_temp.pose[2])*POSE_FREQ
    if rollout_temp.pose[0] > 0.2:
        x_v = -abs(x_v)
    elif rollout_temp.pose[0] < -0.1:
        x_v = abs(x_v)
    if rollout_temp.pose[1] > -0.5:
        y_v = -abs(y_v)
    elif rollout_temp.pose[1] < -0.8:
        y_v = abs(y_v)
    if rollout_temp.pose[2] > 0.495:
        z_v = -abs(z_v)
    elif rollout_temp.pose[2] < 0.435:
        z_v = abs(z_v)
    return x_v, y_v, z_v
    # move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])


def move_callback_position_control(data):
    disp = [data.data[0], data.data[1], data.data[2]]
    move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])          # now it's a specified orientation

def color_callback(color_data):
    rollout_temp.image = bridge.imgmsg_to_cv2(color_data,'bgr8')


def torque_callback(torque_data):
    rollout_temp.torque = [torque_data.joint1,
                           torque_data.joint2,
                           torque_data.joint3,
                           torque_data.joint4,
                           torque_data.joint5,
                           torque_data.joint6,
                           torque_data.joint7
                           ]


def pose_callback(pose_data):
    rollout_temp.pose = [pose_data.pose.position.x,
                         pose_data.pose.position.y,
                         pose_data.pose.position.z
                         ]

    rollout_temp.orientation = [pose_data.pose.orientation.x,
                                pose_data.pose.orientation.y,
                                pose_data.pose.orientation.z,
                                pose_data.pose.orientation.w
                                ]


def joint_callback(joint_data):
    rollout_temp.joint_angle = list(joint_data.position)
    rollout_temp.joint_velocity = list(joint_data.velocity)


def timer_callback(event):
    rollout_observation_image.append(DIR+'/'+str(len(rollout_observation_torque))+'.jpg')
    cv2.imwrite(DIR+'/'+str(len(rollout_observation_torque))+'.jpg', rollout_temp.image)
    rollout_observation_torque.append(rollout_temp.torque)
    rollout_observation_pose.append(rollout_temp.pose)
    rollout_observation_orientation.append(rollout_temp.orientation)
    rollout_observation_joint_angle.append(rollout_temp.joint_angle)
    rollout_observation_joint_velocity.append(rollout_temp.joint_velocity)


if __name__ == '__main__':
    rollout_temp = rollout_data;
    rollout_observation_image = []
    rollout_observation_torque = []
    rollout_observation_pose = []
    rollout_observation_orientation = []
    rollout_observation_joint_angle = []
    rollout_observation_joint_velocity = []

    rospy.init_node('kinova_velo_test')
    rospy.Subscriber('/camera/color/image_raw', Image, color_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/joint_torques', kinova_msgs.msg.JointTorque, torque_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, pose_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/joint_state',sensor_msgs.msg.JointState,joint_callback, queue_size=1)
    if VELOCITY_CONTROL==1:
        rospy.Subscriber('/target_goal', Float32MultiArray, move_callback_velocity_control, queue_size=1)
    else:
        rospy.Subscriber('/target_goal', Float32MultiArray, move_callback_position_control, queue_size=1)

    # DIR1 = "saved_data/"
    DIR1 = ''
    DIR2 = str(rospy.get_time())
    DIR = DIR1+DIR2
    os.makedirs(DIR, mode=0o777)

    velo_pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)
    CURRENT_VELOCITY = [0,0,0,0,0,0]
    velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
    r = rospy.Rate(100)


    rospy.Timer(rospy.Duration(0.1),timer_callback)
    from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform

    start_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    stop_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/stop_force_control', kinova_msgs.srv.Stop)


    move_home()
    while(len(rollout_observation_torque)<DATA_LENGTH):
        if VELOCITY_CONTROL==1:
            CURRENT_VELOCITY = [x_v, y_v, z_v, 0, 0, 0]
            velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
        r.sleep()

    #rospy.Timer.shutdown()
    rollout_observation = [rollout_observation_image,
                           rollout_observation_torque,
                           rollout_observation_pose,
                           rollout_observation_orientation,
                           rollout_observation_joint_angle,
                           rollout_observation_joint_velocity
                           ]
    dataIO = IO(DIR+'/data.pkl')
    dataIO.to_pickle(rollout_observation)

    # data = dataIO.read_pickle()
    # img = cv2.imread(str(data[0][0]))
    # cv2.imshow('1', img)
    # time.sleep(10)
