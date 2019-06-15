#! /usr/bin/env python

import rospy
import tf.transformations as tft

import numpy as np

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg

from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import position_client, move_to_position
from helpers.covariance import generate_cartesian_covariance

import time
import random

MOVING = False  # Flag whether the robot is moving under velocity control.
CURR_Z = 0  # Current end-effector z height.

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
    move_to_position([-0.03, -0.538 , 0.375], [0.072, 0.6902, -0.7172, 0.064])


def move_somewhere():
    move_to_position([-0.03, -0.738 , 0.375],[0.072, 0.6902, -0.7172, 0.064])


def move_rand():
    move_to_position([-0.03+random.uniform(-0.03, 0.03), -0.638+random.uniform(-0.1, 0.1), 0.395+random.uniform(0, 0.1)],[0.072, 0.6902, -0.7172, 0.064])


if __name__ == '__main__':
    rospy.init_node('ggcnn_open_loop_grasp')

   
    from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform

    wrench_sub = rospy.Subscriber('/j2s7s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, robot_wrench_callback, queue_size=1)
    position_sub = rospy.Subscriber('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, robot_position_callback, queue_size=1)

    start_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    stop_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/stop_force_control', kinova_msgs.srv.Stop)
    move_home()
    for i in range(1, 20):
        move_rand()
        time.sleep(5)
