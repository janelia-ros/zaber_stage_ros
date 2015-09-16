#! /usr/bin/env python
# -*- coding: utf-8 -*-
#

import roslib; roslib.load_manifest('zaber_stage')
import rospy

import actionlib

from zaber_stage.msg import EmptyAction,EmptyActionGoal

def home_client():
    client = actionlib.SimpleActionClient('/zaber_stage_node/home', EmptyAction)

    client.wait_for_server()

    goal = EmptyActionGoal()

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('zaber_stage_home')
        rospy.loginfo('zaber_stage homing...')
        result = home_client()
        rospy.loginfo('zaber_stage home finished!')
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
