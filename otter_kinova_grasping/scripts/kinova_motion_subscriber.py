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

import inspect


currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
grandgrandparentdir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(currentdir))))

VELOCITY_CONTROL = 1            # when 0, position control; when 1, velocity control
DATA_LENGTH = 50              # set the total data length
POSE_FREQ = 10                   # the frequency of input
ROLLOUT_AMOUNT = 2          # set the number of rollout sets

bridge = CvBridge()

x_v = 0
y_v = 0
z_v = 0

global i
global j
i = 1
j = 1




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
    action = []
    cmd = []


def move_home():
    global x_r
    global y_r
    global z_r
    # x_r = -0.03
    # y_r = -0.538
    # z_r = 0.375
    # noinspection PyInterpreter
    x_r = 0.09
    y_r = -0.446
    z_r = 0.375
    # move_to_position([x_r,y_r,z_r], [0.072, 0.6902, -0.7172, 0.064])
    move_to_position([x_r, y_r, z_r], [0.708, -0.019, 0.037, 0.705])
    time.sleep(0)


def move_callback_velocity_control(data):
    # disp = [data.data[0], data.data[1], data.data[2]]
    global x_v
    global y_v
    global z_v
    x_v = data.data[0]*POSE_FREQ
    y_v = data.data[1]*POSE_FREQ
    z_v = data.data[2]*POSE_FREQ
    rollout_temp.action = data.data
    rollout_temp.cmd = [x_v, y_v, z_v, 0, 0, 0]
    if rollout_temp.pose[0] > 0.2:
        x_v = -abs(x_v)
    elif rollout_temp.pose[0] < -0.1:
        x_v = abs(x_v)
    if rollout_temp.pose[1] > -0.4:
        y_v = -abs(y_v)
    elif rollout_temp.pose[1] < -0.6:
        y_v = abs(y_v)
    if rollout_temp.pose[2] > 0.495:
        z_v = -abs(z_v)
    elif rollout_temp.pose[2] < 0.435:
        z_v = abs(z_v)
    return x_v, y_v, z_v
    # move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])


def move_callback_position_control(data):
    disp = [data.data[0], data.data[1], data.data[2]]
    # move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])          # now it's a specified orientation
    move_to_position(disp, [0.708, -0.019, 0.037, 0.705])  # now it's a specified orientation

def color_callback(color_data):

    original_image = bridge.imgmsg_to_cv2(color_data,'bgr8')

    # Crop a square out of the middle of the depth and resize it to 300*300
    crop_size = 480
    rollout_temp.image = cv2.resize(original_image[(480 - crop_size) // 2:(480 - crop_size) // 2 + crop_size,
                            (640 - crop_size) // 2:(640 - crop_size) // 2 + crop_size], (480, 480))


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
    global rollout_flag
    global i
    global j
    if rollout_flag==1:
        rollout_observation_image.append(DIR+'/'+str(len(rollout_observation_torque))+'.jpg')
        cv2.imwrite(DIR+'/'+str(len(rollout_observation_torque)//50+1)+'_'+str(len(rollout_observation_torque)%50+1)+'.jpg', rollout_temp.image)
        rollout_observation_torque.append(rollout_temp.torque)
        rollout_observation_pose.append(rollout_temp.pose)
        rollout_observation_orientation.append(rollout_temp.orientation)
        rollout_observation_joint_angle.append(rollout_temp.joint_angle)
        rollout_observation_joint_velocity.append(rollout_temp.joint_velocity)
        rollout_observation_joint_action.append(rollout_temp.action)
        rollout_observation_joint_cmd.append(rollout_temp.cmd)

if __name__ == '__main__':
    rollout_temp = rollout_data
    rollout_observation_image = []
    rollout_observation_torque = []
    rollout_observation_pose = []
    rollout_observation_orientation = []
    rollout_observation_joint_angle = []
    rollout_observation_joint_velocity = []
    rollout_observation_joint_action = []
    rollout_observation_joint_cmd = []

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
    DIR1 = grandgrandparentdir
    DIR2 = str(rospy.get_time())
    DIR = DIR1+'/data/'+DIR2
    os.makedirs(DIR, mode=0o777)

    velo_pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)
    CURRENT_VELOCITY = [0,0,0,0,0,0]
    velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
    r = rospy.Rate(100)

    rospy.Timer(rospy.Duration(0.1),timer_callback)
    from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform

    start_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    stop_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/stop_force_control', kinova_msgs.srv.Stop)

    rollout_flag = 0  # when 0, do not record, when 1, keep recording
    while (len(rollout_observation_torque)<ROLLOUT_AMOUNT*DATA_LENGTH):
        global rollout_flag
        move_home()
        time.sleep(0.5)
        rollout_flag = 1
        print('@')
        print(len(rollout_observation_torque))
        print(DATA_LENGTH)
        if (len(rollout_observation_torque)%DATA_LENGTH)==0:
            rollout_flag2 = 1
            print('@#')
        while((len(rollout_observation_torque)%DATA_LENGTH)!=0 or rollout_flag2==1):
            if VELOCITY_CONTROL==1:
                CURRENT_VELOCITY = [x_v, y_v, z_v, 0, 0, 0]
                velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
            r.sleep()
            print(len(rollout_observation_torque))
        rollout_flag2 = 0
        rollout_flag = 0
        print('!')

    print("345")
    #rospy.Timer.shutdown()
    rollout_observation = [
                           np.array(rollout_observation_joint_action),
                           np.array(rollout_observation_joint_cmd),
                           np.array(rollout_observation_torque),
                           np.array(rollout_observation_pose),
                           np.array(rollout_observation_orientation),
                           np.array(rollout_observation_joint_angle),
                           np.array(rollout_observation_joint_velocity)
                           ]
    dataIO = IO(DIR+'/data.pkl')
    dataIO.to_pickle(rollout_observation)

    # data = dataIO.read_pickle()
    # img = cv2.imread(str(data[0][0]))
    # cv2.imshow('1', img)
    # time.sleep(10)
