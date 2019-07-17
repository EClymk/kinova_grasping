"""
Use this script to control the env with your keyboard.
For this script to work, you need to have the PyGame window in focus.
See/modify `char_to_action` to set the key-to-action mapping.
"""
import sys

import numpy as np

import pygame
from pygame.locals import QUIT, KEYDOWN
print('asdfv')

import math
import numpy as np
import time
from pkg_resources import parse_version
import os
import rospy
from std_msgs.msg import Float32MultiArray

position_target_pub = rospy.Publisher('/target_goal', Float32MultiArray, queue_size=1)
rospy.init_node('kinova_random_publisher')
r = rospy.Rate(10)

pygame.init()
screen = pygame.display.set_mode((400, 300))


char_to_action = {
    'w': np.array([0, -1, 0, 0]),
    'a': np.array([1, 0, 0, 0]),
    's': np.array([0, 1, 0, 0]),
    'd': np.array([-1, 0, 0, 0]),

    'q': np.array([1, -1, 0, 0]),
    'e': np.array([-1, -1, 0, 0]),
    'z': np.array([1, 1, 0, 0]),
    'c': np.array([-1, 1, 0, 0]),
    'k': np.array([0, 0, 1, 0]),
    'j': np.array([0, 0, -1, 0]),


    'h': 'close',
    'l': 'open',
    'x': 'toggle',
    'r': 'reset',
    'p': 'put obj in hand',
}
import pygame

import numpy as np

env_params = {
    "environment_name": "ImageKinovaReacherXYZEnv-v0",  #Reach
    "random_start": False,
    "random_target": False,

    "image": True,
    "image_dim": 128,
    "goal_point": [0.5, 0, 0.5],
    'isAbsolute_control': False,
    "reward_test":1.66

}
data_params=dict(
        num_rollouts=10,
        init_std=0.5,
        smooth_noise=False,
    )
horizon = 50

env = gym.from_config(env_params)

NDIM = env.gym_env.action_space.low.size
lock_action = False
obs = env.reset()
action = np.zeros(10)
while True:
    done = False
    if not lock_action:
        action[:3] = 0
    for event in pygame.event.get():
        event_happened = True
        if event.type == QUIT:
            sys.exit()
        if event.type == KEYDOWN:
            char = event.dict['key']
            new_action = char_to_action.get(chr(char), None)
            if new_action == 'toggle':
                lock_action = not lock_action
            elif new_action == 'reset':
                done = True
            elif new_action == 'close':
                action[3] = 1
            elif new_action == 'open':
                action[3] = -1
            elif new_action == 'put obj in hand':
                print("putting obj in hand")
                env.put_obj_in_hand()
                action[3] = 1
            elif new_action is not None:
                action[:3] = new_action[:3]
            else:
                action = np.zeros(3)
    print('______________________')
    print('actions: ', action)
    cmd_msg = Float32MultiArray()
    cmd_msg.data = action[:3]
    position_target_pub.publish(cmd_msg)
    r.sleep()
