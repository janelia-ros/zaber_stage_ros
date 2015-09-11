#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function, division

from zaber_device import ZaberStage

import rospy
from geometry_msgs.msg import Twist,Pose


class ZaberStageNode(object):
    def __init__(self,*args,**kwargs):
        rospy.loginfo('Initializing zaber_stage_node...')
        self._initialized = False
        self._rate = rospy.Rate(10) # 10hz
        self._pub = rospy.Publisher('/stage/pose', Pose, queue_size=10)
        self._sub = rospy.Subscriber('/stage/cmd_vel',Twist,self.cmd_vel_callback)
        x_serial_number = rospy.get_param('/stage/x_serial_number', None)
        x_alias = rospy.get_param('/stage/x_alias', None)
        y_serial_number = rospy.get_param('/stage/y_serial_number', None)
        y_alias = rospy.get_param('/stage/y_alias', None)
        z_serial_number = rospy.get_param('/stage/z_serial_number', None)
        z_alias = rospy.get_param('/stage/z_alias', None)
        self._stage = ZaberStage()
        if (x_serial_number is not None) and (x_alias is not None):
            self._stage.set_x_axis(x_serial_number,x_alias)
        if (y_serial_number is not None) and (y_alias is not None):
            self._stage.set_y_axis(y_serial_number,y_alias)
        if (z_serial_number is not None) and (z_alias is not None):
            self._stage.set_y_axis(z_serial_number,z_alias)
        rospy.loginfo('zaber_stage_node initialized!')
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
        rospy.init_node('zaber_stage_node')
        zsn = ZaberStageNode()
        zsn.publish_pose()
    except rospy.ROSInterruptException:
        pass
