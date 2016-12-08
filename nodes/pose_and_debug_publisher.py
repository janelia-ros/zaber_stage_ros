#! /usr/bin/env python
# -*- coding: utf-8 -*-
#

import roslib; roslib.load_manifest('zaber_stage')
import rospy
import actionlib

from zaber_stage.msg import PoseAndDebugInfo
from zaber_stage.srv import GetPoseAndDebugInfo,GetPoseAndDebugInfoResponse

def pose_publisher():
    rospy.init_node('zaber_stage_pose_and_debug_publisher')
    rospy.loginfo('zaber_stage pose_and_debug_publisher...')
    rate = rospy.Rate(4)
    pub_pose = rospy.Publisher('/zaber_stage_node/pose',Pose,queue_size=10)
    pub_pose_and_debug = rospy.Publisher('/zaber_stage_node/pose_and_debug_info',PoseAndDebugInfo,queue_size=10)
    rospy.wait_for_service('/zaber_stage_node/get_pose_and_debug_info')
    get_pose = rospy.ServiceProxy('/zaber_stage_node/get_pose_and_debug_info',GetPoseAndDebugInfo)
    while not rospy.is_shutdown():
        try:
            response = get_pose_and_debug_info()
            pub_pose.publish(response.pose_and_debug_info.pose)
            pub_pose_and_debug.publish(response.pose_and_debug_info)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
