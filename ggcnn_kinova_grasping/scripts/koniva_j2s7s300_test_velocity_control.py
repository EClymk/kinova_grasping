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

import time
import random
import pickle
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

MOVING = False  # Flag whether the robot is moving under velocity control.
CURR_Z = 0  # Current end-effector z height.
bridge = CvBridge()

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


rollout_temp = rollout_data;


def robot_wrench_callback(msg):
    # Monitor wrench to cancel movement on collision.
    global MOVING
    if MOVING and msg.wrench.force.z < -2.0:
        MOVING = False
        rospy.logerr('Force Detected. Stopping.')


def robot_position_callback(msg):
    # Monitor robot position.
    global CURR_Z
    CURR_Z = msg.pose.position.z


def move_home():
    global x_r
    global y_r
    global z_r
    x_r = -0.03
    y_r = -0.538
    z_r = 0.375
    move_to_position([x_r,y_r,z_r], [0.072, 0.6902, -0.7172, 0.064])
    time.sleep(5)


def move_somewhere():
    move_to_position([-0.03, -0.738 , 0.375],[0.072, 0.6902, -0.7172, 0.064])


def feed_rand():
    x_r_feed = random.uniform(-0.01, 0.01)
    y_r_feed = random.uniform(-0.01, 0.01)
    z_r_feed = random.uniform(-0.01, 0.01)
    return x_r_feed,y_r_feed,z_r_feed


def move_rand():
    global x_r
    global y_r
    global z_r
    x_r_feed, y_r_feed, z_r_feed = feed_rand()
    # x_r = -0.03+random.uniform(-0.03, 0.03)
    x_r = x_r+x_r_feed
    if x_r > 0.2:
        x_r = 0.2
    elif x_r < -0.1:
        x_r = -0.1
    # y_r = -0.638+random.uniform(-0.05, 0.05)
    y_r = y_r+y_r_feed
    if y_r > -0.5:
        y_r = -0.5
    elif y_r < -0.8:
        y_r = -0.8
    # z_r = 0.395+random.uniform(0, 0.1)
    z_r = z_r+z_r_feed
    if z_r > 0.495:
        z_r = 0.495
    elif z_r < 0.395:
        z_r = 0.395
    move_to_position([x_r, y_r, z_r],[0.072, 0.6902, -0.7172, 0.064])



def color_callback(color_data):
    rollout_temp.image = bridge.imgmsg_to_cv2(color_data,'bgr8')
    # print(color_data.height)
    # print(color_data.width)
    # print(color_data.is_bigendian)
    # print(color_data.step)
    # print(color_data.encoding)

    # print('color')
    # print(type(color_data))
    # print('\n')

def torque_callback(torque_data):
    rollout_temp.torque = [torque_data.joint1,
                           torque_data.joint2,
                           torque_data.joint3,
                           torque_data.joint4,
                           torque_data.joint5,
                           torque_data.joint6,
                           torque_data.joint7
                           ]
    # print(torque_data)
    # print('\n')

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
    # print(pose_data)
    # print('\n')

def joint_callback(joint_data):
    rollout_temp.joint_angle = list(joint_data.position)
    rollout_temp.joint_velocity = list(joint_data.velocity)
    #print(joint_data)
    #print('\n')


def timer_callback(event):
    # print(rollout_temp.image)
    # print(time.time())
    # print(rollout_temp.torque)
    # print(rollout_temp.pose)
    # print(rollout_temp.orientation)
    # print(rollout_temp.joint_angle)
    # print(rollout_temp.joint_velocity)
    rollout_observation_image.append(rollout_temp.image)
    rollout_observation_torque.append(rollout_temp.torque)
    rollout_observation_pose.append(rollout_temp.pose)
    rollout_observation_orientation.append(rollout_temp.orientation)
    rollout_observation_joint_angle.append(rollout_temp.joint_angle)
    rollout_observation_joint_velocity.append(rollout_temp.joint_velocity)


if __name__ == '__main__':
    rollout_observation_image = []
    rollout_observation_torque = []
    rollout_observation_pose = []
    rollout_observation_orientation = []
    rollout_observation_joint_angle = []
    rollout_observation_joint_velocity = []

    rospy.init_node('ggcnn_open_loop_grasp')
    rospy.Subscriber('/camera/color/image_raw', Image, color_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/joint_torques', kinova_msgs.msg.JointTorque, torque_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, pose_callback, queue_size=1)
    rospy.Subscriber('/j2s7s300_driver/out/joint_state',sensor_msgs.msg.JointState,joint_callback, queue_size=1)
    # velo_pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)
    # CURRENT_VELOCITY = [0,0,0,0,0,0]
    # velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
    # r = rospy.Rate(100)


    rospy.Timer(rospy.Duration(0.1),timer_callback)
    from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform

    wrench_sub = rospy.Subscriber('/j2s7s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, robot_wrench_callback, queue_size=1)
    position_sub = rospy.Subscriber('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, robot_position_callback, queue_size=1)

    start_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    stop_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/stop_force_control', kinova_msgs.srv.Stop)
    move_home()
    for i in range(0, 100):
        move_rand()
        velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))

        # time.sleep(0.2)

    #rospy.Timer.shutdown()
    rollout_observation = [rollout_observation_image,
                           rollout_observation_torque,
                           rollout_observation_pose,
                           rollout_observation_orientation,
                           rollout_observation_joint_angle,
                           rollout_observation_joint_velocity
                           ]
    print('!!')
    # dataIO = IO('data.pkl')
    # dataIO.to_pickle(rollout_observation)
    time.sleep(2)

    # data = dataIO.read_pickle()
    # print(type(data[0][0]))
    # print(len(data[0][0]))
    # image = np.reshape(data[0][0],(3,640,480))
    # np.nbarrayObject.astype(int)
    # cv2.imshow('img',data[0][0])
    # cv2.imwrite('/home/hu/test.bmp',data[0][0])
    # time.sleep(10)
    # print(data[0][0])