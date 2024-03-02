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
    rospy.init_node('goal_creator')
    client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    client.wait_for_server()
    pose = Pose()

    # Create Goal 1
    pose.position.x = 0.40
    pose.position.y = 0.8
    pose.position.z = 0.0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 1
    
    # Setup goal to send to client
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_footprint'
    goal.target_pose.pose = pose
    client.send_goal(goal)
    client.wait_for_result()
    print("Goal 1 Reached")

    # Create Goal 2
    pose2 = Pose()
    pose2.position.x = -0.5
    pose2.position.y = -0.5
    pose2.position.z = 0.0
    pose2.orientation.x = 0
    pose2.orientation.y = 0
    pose2.orientation.z = 1

    # Setup goal 2 to send to client
    goal2 = MoveBaseGoal()
    goal2.target_pose.header.frame_id = 'base_footprint'
    goal2.target_pose.pose = pose2
    client.send_goal(goal)
    client.wait_for_result()
    print("Goal 2 Reached")
    
        