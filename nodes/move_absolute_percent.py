#! /usr/bin/env python
# -*- coding: utf-8 -*-
#

import roslib; roslib.load_manifest('zaber_stage')
import rospy

import actionlib

from geometry_msgs.msg import Pose
from zaber_stage.msg import MoveAction,MoveGoal

def move_absolute_client():
    client = actionlib.SimpleActionClient('/zaber_stage_node/move_absolute_percent', MoveAction)

    client.wait_for_server()

    pose = Pose()
    x = rospy.get_param('~x', None)
    if x is not None:
        pose.position.x = x
    y = rospy.get_param('~y', None)
    if y is not None:
        pose.position.y = y
    goal = MoveGoal(pose=pose)

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('zaber_stage_move_absolute_percent')
        rospy.loginfo('zaber_stage moving absolute percent...')
        result = move_absolute_client()
        rospy.loginfo('zaber_stage move absolute percent finished!')
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
