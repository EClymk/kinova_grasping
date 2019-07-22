
#! /usr/bin/env python

import rospy
#import tf.transformations as tft

import numpy as np

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg

from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import position_client, move_to_position
from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
from helpers.covariance import generate_cartesian_covariance


if __name__ == '__main__':
    #rospy.init_node('ggcnn_open_loop_grasp')


    # https://github.com/dougsm/rosbag_recording_services
    # start_record_srv = rospy.ServiceProxy('/data_recording/start_recording', std_srvs.srv.Trigger)
    # stop_record_srv = rospy.ServiceProxy('/data_recording/stop_recording', std_srvs.srv.Trigger)

    # Enable/disable force control.
    start_force_srv = rospy.ServiceProxy('/j2n6s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    stop_force_srv = rospy.ServiceProxy('/j2n6s300_driver/in/stop_force_control', kinova_msgs.srv.Stop)

    # Home position.
    move_to_position([0, -0.38, 0.25], [0.99, 0, 0, np.sqrt(1-0.99**2)])

    rospy.sleep(5)

    while not rospy.is_shutdown():

        rospy.sleep(0.5)
        set_finger_positions([0, 0])
        rospy.sleep(0.5)

        raw_input('Press Enter to Start.')

        # start_record_srv(std_srvs.srv.TriggerRequest())
        rospy.sleep(0.5)
        #execute_grasp()
        move_to_position([0, -0.38, 0.25], [0.99, 0, 0, np.sqrt(1-0.99**2)])
        rospy.sleep(0.5)
        # stop_record_srv(std_srvs.srv.TriggerRequest())

        raw_input('Press Enter to Complete')
