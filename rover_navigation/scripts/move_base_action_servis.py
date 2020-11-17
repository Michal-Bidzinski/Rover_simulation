#!/usr/bin/env python
import argparse
import roslib  
import rospy
import actionlib

#move_base_msgs
from move_base_msgs.msg import *

def simple_move(coords):

    #Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )

    #create goal
    goal = MoveBaseGoal()

    #use self?
    #set goal
    c = coords
    goal.target_pose.pose.position.x = c[0]
    goal.target_pose.pose.position.y = c[1]
    goal.target_pose.pose.orientation.w = c[2]
    goal.target_pose.header.frame_id = 'odom'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    sac.wait_for_server()

    #send goal
    sac.send_goal(goal)

    #finish
    sac.wait_for_result()

    #print result
    print(sac.get_result())

