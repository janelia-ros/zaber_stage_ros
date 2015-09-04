#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function, division

from zaber_stage import ZaberStage

import rospy
from geometry_msgs.msg import Twist,Pose


class ZaberStageNode(object):
    def __init__(self,*args,**kwargs):
        print('Initializing...')
        self._initialized = False
        rospy.init_node('zaber_stage_node', anonymous=True)
        self._rate = rospy.Rate(10) # 10hz
        self._pub = rospy.Publisher('/stage/pose', Pose, queue_size=10)
        self._sub = rospy.Subscriber('/stage/cmd_vel',Twist,self.cmd_vel_callback)
        self._stage = ZaberStage()
        self._stage.set_x_axis(123,10)
        self._stage.set_y_axis(123,11)
        print('Initialized!')
        self._initialized = True

    def publish_pose(self):
        while not self._initialized:
            self._rate.sleep()
        while not rospy.is_shutdown():
            pose = Pose()
            positions = self._stage.get_positions()
            pose.position.x = positions[0]
            pose.position.y = positions[1]
            pose.position.z = positions[2]
            self._pub.publish(pose)
            self._rate.sleep()

    def cmd_vel_callback(self,data):
        if self._initialized:
            x_vel = data.linear.x
            y_vel = data.linear.y
            z_vel = data.linear.z
            self._stage.move_x_at_speed(x_vel)
            self._stage.move_y_at_speed(y_vel)
            self._stage.move_z_at_speed(z_vel)

if __name__ == '__main__':
    try:
        zsn = ZaberStageNode()
        zsn.publish_pose()
    except rospy.ROSInterruptException:
        pass
