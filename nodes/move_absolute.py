#! /usr/bin/env python
# -*- coding: utf-8 -*-
#

import roslib; roslib.load_manifest('zaber_stage')
import rospy

import actionlib

from geometry_msgs.msg import Pose
from zaber_stage.msg import MoveAction,MoveGoal

def move_absolute_client():
    client = actionlib.SimpleActionClient('/zaber_stage_node/move_absolute', MoveAction)

    client.wait_for_server()

    pose = Pose()
    pose.position.x = 1000
    pose.position.y = 2000
    goal = MoveGoal(pose=pose)

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('zaber_stage_move_absolute')
        rospy.loginfo('zaber_stage moving absolute...')
        result = move_absolute_client()
        rospy.loginfo('zaber_stage move absolute finished!')
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
