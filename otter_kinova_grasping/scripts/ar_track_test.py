#! /usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('ar_tf_listener')
    listener = tf.TransformListener()
    ar_pos_pub = rospy.Publisher('/ar_tag/pose/marker_15', geometry_msgs.msg.Pose,
                                    queue_size=1)
    r=rospy.Rate(50)
    listener.waitForTransform("/j2s7s300_link_base", "/ar_marker_15", rospy.Time(), rospy.Duration(100))
    while 1:
        r.sleep
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/j2s7s300_link_base", "/ar_marker_15", now, rospy.Duration(100))
            (trans, rot) = listener.lookupTransform("/j2s7s300_link_base", "/ar_marker_15", now)
            print(trans)
            print(rot)
            ar_pos_pub.publish(trans, rot)

        except (tf.LookupException, tf.ConnectivityException):
            print('err')