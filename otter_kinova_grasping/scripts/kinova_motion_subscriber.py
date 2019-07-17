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
import msgs.ActionCommand.msg
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

x_v = 0
y_v = 0
z_v = 0
temp_angles = []
home_angles = []

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
    torque = [0,0,0,0,0,0,0]
    pose = [0,0,0]
    orientation = [0,0,0]
    joint_angle = [0,0,0,0,0,0,0]
    joint_velocity = [0,0,0,0,0,0,0]
    action = []
    cmd = []
    reward = []


def robot_wrench_callback(msg):
    global MOVING
    if abs(msg.wrench.force.x) > 5.0 or abs(msg.wrench.force.y) > 5.0 or abs(msg.wrench.force.z) > 7.0:
        rospy.logerr('Force Detected. Stopping.')
        MOVING = False
        print(msg.wrench)
        print(temp_angles)
        if temp_angles!=[]:
                joint_angle_client([temp_angles.joint1, temp_angles.joint2, temp_angles.joint3, temp_angles.joint4,
                                   temp_angles.joint5, temp_angles.joint6, temp_angles.joint7])
                rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)




def move_home():
    joint_angle_client([home_angles.joint1, home_angles.joint2, home_angles.joint3, home_angles.joint4,
                                   home_angles.joint5, home_angles.joint6, home_angles.joint7])              # should be in absolute degree
    time.sleep(0.5)


def move_home_init():
    global x_r                         # using position control, however, may result in different pose
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
    x_v = data.data[0]*POSE_FREQ*K
    y_v = data.data[1]*POSE_FREQ*K
    z_v = data.data[2]*POSE_FREQ*K
    rollout_temp.action = data.data
    rollout_temp.cmd = [x_v, y_v, z_v, 0, 0, 0]
    if rollout_temp.pose[0] > 0.2:
        x_v = -abs(x_v)
    elif rollout_temp.pose[0] < -0.1:
        x_v = abs(x_v)
    if rollout_temp.pose[1] > -0.4:
        y_v = -abs(y_v)
    elif rollout_temp.pose[1] < -0.7:
        y_v = abs(y_v)
    if rollout_temp.pose[2] > 0.465:
        z_v = -abs(z_v)
    elif rollout_temp.pose[2] < 0.365:
        z_v = abs(z_v)
    return x_v, y_v, z_v
    # move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])


def move_callback_position_control(data):
    disp =  rollout_temp.pose
    for i in len(rollout_temp.pose):
        rollout_temp.pose[i] += data[i]*K
    # move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])          # now it's a specified orientation
    move_to_position(disp, [0.708, -0.019, 0.037, 0.705])  # now it's a specified orientation

def color_callback(color_data):

    original_image = bridge.imgmsg_to_cv2(color_data,'bgr8')

    # Crop a square out of the middle of the depth and resize it to 300*300
    crop_size = 480
    rollout_temp.image = cv2.resize(original_image[(480 - crop_size) // 2:(480 - crop_size) // 2 + crop_size,
                            (640 - crop_size) // 2:(640 - crop_size) // 2 + crop_size], (480, 480))


def jointangle_callback(data):
    global temp_angles
    temp_angles = data
    print('!!')


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


def reward(target_position, current_position):
    pose_data = -np.square(np.array(target_position)-np.array(current_position))
    return pose_data


def timer_callback(event):
    global rollout_flag
    global i
    global j
    if rollout_flag==1:
        rollout_observation_image.append(DIR+'/'+str(len(rollout_observation_torque))+'.jpg')
        cv2.imwrite(DIR+'/'+str(len(rollout_observation_torque)//50+1)+'_'+str(len(rollout_observation_torque)%50+1)+'.jpg',rollout_temp.image)
        rollout_observation_torque.append(rollout_temp.torque)
        rollout_observation_pose.append(rollout_temp.pose)
        rollout_observation_orientation.append(rollout_temp.orientation)
        rollout_observation_joint_angle.append(rollout_temp.joint_angle)
        rollout_observation_joint_velocity.append(rollout_temp.joint_velocity)
        rollout_observation_action.append(rollout_temp.action)
        rollout_observation_cmd.append(rollout_temp.cmd)
        rollout_observation_reward.append(reward(target_position,rollout_temp.pose))

if __name__ == '__main__':
    rollout_temp = rollout_data
    rollout_observation_image = []
    rollout_observation_torque = []
    rollout_observation_pose = []
    rollout_observation_orientation = []
    rollout_observation_joint_angle = []
    rollout_observation_joint_velocity = []
    rollout_observation_action = []
    rollout_observation_cmd = []
    rollout_observation_reward = []

    rospy.init_node('kinova_velo_test')
    rospy.Subscriber('/j2s7s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, robot_wrench_callback,
                     queue_size=1)
    rospy.Subscriber('/camera/color/image_raw', Image, color_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/joint_torques', kinova_msgs.msg.JointTorque, torque_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, pose_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/joint_state', sensor_msgs.msg.JointState, joint_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, jointangle_callback, queue_size=1)
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
    time.sleep(1)
    stop_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start', kinova_msgs.srv.Start)
    print('home')
    home_srv = rospy.ServiceProxy('/j2s7s300_driver/in/home_arm', kinova_msgs.srv.HomeArm)
    home_srv()
    time.sleep(15)


    rollout_flag = 0
    rollout_flag = 0  # when 0, do not record, when 1, keep recording
    move_home_init()
    time.sleep(2)
    # home_angles = temp_angles

    while (len(rollout_observation_torque)<ROLLOUT_AMOUNT*DATA_LENGTH) & MOVING:
        move_home()
        time.sleep(2)
        rollout_flag = 1
        if (len(rollout_observation_torque)%DATA_LENGTH)==0 & MOVING:
            rollout_flag2 = 1
        while((len(rollout_observation_torque)%DATA_LENGTH)!=0 or rollout_flag2==1) & MOVING:
            if VELOCITY_CONTROL==1:
                CURRENT_VELOCITY = [x_v, y_v, z_v, 0, 0, 0]
                velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
            r.sleep()
            if (len(rollout_observation_torque)%DATA_LENGTH)!=0:
                rollout_flag2 = 0
        rollout_flag = 0
        move_home()

    time.sleep(3)

    #rospy.Timer.shutdown()
    rollout_observation = [
                           np.array(rollout_observation_action).reshape((ROLLOUT_AMOUNT,DATA_LENGTH,3)),
                           np.array(rollout_observation_cmd).reshape((ROLLOUT_AMOUNT,DATA_LENGTH,6)),
                           np.array(rollout_observation_reward).reshape((ROLLOUT_AMOUNT, DATA_LENGTH, 3)),
                           np.array(rollout_observation_torque).reshape((ROLLOUT_AMOUNT,DATA_LENGTH,7)),
                           np.array(rollout_observation_pose).reshape((ROLLOUT_AMOUNT,DATA_LENGTH,3)),
                           np.array(rollout_observation_orientation).reshape((ROLLOUT_AMOUNT,DATA_LENGTH,4)),
                           np.array(rollout_observation_joint_angle).reshape((ROLLOUT_AMOUNT,DATA_LENGTH,10)),
                           np.array(rollout_observation_joint_velocity).reshape((ROLLOUT_AMOUNT,DATA_LENGTH,10))
                           ]
    dataIO = IO(DIR+'/data.pkl')
    dataIO.to_pickle(rollout_observation)

    # data = dataIO.read_pickle()
    # img = cv2.imread(str(data[0][0]))
    # cv2.imshow('1', img)
    # time.sleep(10)
