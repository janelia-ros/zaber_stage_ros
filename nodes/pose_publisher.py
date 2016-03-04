#! /usr/bin/env python
# -*- coding: utf-8 -*-
#

import roslib; roslib.load_manifest('zaber_stage')
import rospy
import actionlib

from geometry_msgs.msg import Pose
from zaber_stage.srv import GetPose,GetPoseResponse

def pose_publisher():
    rospy.init_node('zaber_stage_pose_publisher')
    rospy.loginfo('zaber_stage pose_publisher...')
    rate = rospy.Rate(4)
    pub = rospy.Publisher('/zaber_stage_node/pose',Pose,queue_size=10)
    rospy.wait_for_service('/zaber_stage_node/get_pose')
    get_pose = rospy.ServiceProxy('/zaber_stage_node/get_pose',GetPose)
    while not rospy.is_shutdown():
        try:
            response = get_pose()
            pub.publish(response.pose)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
