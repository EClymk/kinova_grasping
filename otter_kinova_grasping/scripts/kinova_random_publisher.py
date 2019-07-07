#! /usr/bin/env python

import rospy
# import Position_target.msg
from std_msgs.msg import Float32MultiArray
import random

x_r = -0.09
y_r = -0.446
z_r = 0.375

position_target_pub = rospy.Publisher('/target_goal', Float32MultiArray, queue_size=1)
rospy.init_node('kinova_random_publisher')
r = rospy.Rate(10)

def feed_rand():
    x_r_feed = random.uniform(-1, 1)
    y_r_feed = random.uniform(-1, 1)
    z_r_feed = random.uniform(-1, 1)
    return x_r_feed, y_r_feed, z_r_feed


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
    if y_r > -0.4:
        y_r = -0.4
    elif y_r < -0.7:
        y_r = -0.7
    # z_r = 0.395+random.uniform(0, 0.1)
    z_r = z_r+z_r_feed
    if z_r > 0.45:
        z_r = 0.45
    elif z_r < 0.38:
        z_r = 0.38
    # x_r = 0
    # y_r = -0.6
    # z_r = 0.45
    return x_r, y_r, z_r


# def move_rand():
#     global x_v
#     global y_v
#     global z_v
#     global CURRENT_VELOCITY
#     x_r_feed, y_r_feed, z_r_feed = feed_rand()
#     x_v = x_v+x_r_feed
#     if rollout_temp.pose[0] > 0.2:
#         x_v = -abs(x_v)
#     elif rollout_temp.pose[0] < -0.1:
#         x_v = abs(x_v)
#     y_v = y_v+y_r_feed
#     if rollout_temp.pose[1] > -0.5:
#         y_v = -abs(y_v)
#     elif rollout_temp.pose[1] < -0.8:
#         y_v = abs(y_v)
#     z_v = z_v+z_r_feed
#     if rollout_temp.pose[2] > 0.495:
#         z_v = -abs(z_v)
#     elif rollout_temp.pose[2] < 0.395:
#         z_v = abs(z_v)
#     return x_v, _v, z_v
while not rospy.is_shutdown():
    cmd_msg = Float32MultiArray()
    x, y, z = feed_rand()
    print(x,y,z)
    cmd_msg.data = [x, y, z, 0, 0, 0]
    position_target_pub.publish(cmd_msg)
    r.sleep()