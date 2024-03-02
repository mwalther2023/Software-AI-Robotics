#!/usr/bin/env python3

from tf import TransformListener
import tf
import math
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from threading import Thread

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    # Create Transform Listener
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Perform lookupTransform for turtle
            (pos,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue 
        # Log position info to console
        rospy.loginfo(pos)
        rate.sleep()
    
        