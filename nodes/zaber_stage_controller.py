#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function,division

from zaber_device import ZaberStage

import rospy
from geometry_msgs.msg import Twist,Pose
import actionlib
from std_msgs.msg import Empty

from zaber_stage.srv import GetPose,GetPoseResponse
from zaber_stage.srv import Moving,MovingResponse

from zaber_stage.msg import EmptyAction,EmptyFeedback


class ZaberStageController(object):
    def __init__(self,*args,**kwargs):
        rospy.loginfo('Initializing zaber_stage_controller...')
        self._initialized = False
        self._rate = rospy.Rate(1)

        self._cmd_vel_sub = rospy.Subscriber('~cmd_vel',Twist,self._cmd_vel_callback)
        self._stop_x_sub = rospy.Subscriber('~stop_x',Empty,self._stop_x_callback)
        self._stop_y_sub = rospy.Subscriber('~stop_y',Empty,self._stop_y_callback)
        self._stop_z_sub = rospy.Subscriber('~stop_z',Empty,self._stop_z_callback)
        self._get_pose_srv = rospy.Service('~get_pose',GetPose,self._get_pose_callback)
        self._moving_srv = rospy.Service('~moving',Moving,self._moving_callback)
        self._home_action = actionlib.SimpleActionServer('~home', EmptyAction, self._home_callback, False)

        x_serial_number = rospy.get_param('~x_serial_number', None)
        if x_serial_number == '':
            x_serial_number = None
        x_alias = rospy.get_param('~x_alias', None)
        if x_alias == '':
            x_alias = None
        y_serial_number = rospy.get_param('~y_serial_number', None)
        if y_serial_number == '':
            y_serial_number = None
        y_alias = rospy.get_param('~y_alias', None)
        if y_alias == '':
            y_alias = None
        z_serial_number = rospy.get_param('~z_serial_number', None)
        if z_serial_number == '':
            z_serial_number = None
        z_alias = rospy.get_param('~z_alias', None)
        if z_alias == '':
            z_alias = None
        axis_set = False
        if (x_serial_number is not None) and (x_alias is not None):
            axis_set = True
        if (y_serial_number is not None) and (y_alias is not None):
            axis_set = True
        if (z_serial_number is not None) and (z_alias is not None):
            axis_set = True
        if not axis_set:
            err_str = "Not enough zaber_stage_controller axis arguments specified! (x_serial_number,x_alias,y_serial_number,y_alias,z_serial_number,z_alias)"
            rospy.signal_shutdown(err_str)
            rospy.logerr(err_str)
        else:
            self._stage = ZaberStage()
            if (x_serial_number is not None) and (x_alias is not None):
                self._stage.set_x_axis(x_serial_number,x_alias)
            if (y_serial_number is not None) and (y_alias is not None):
                self._stage.set_y_axis(y_serial_number,y_alias)
            if (z_serial_number is not None) and (z_alias is not None):
                self._stage.set_z_axis(z_serial_number,z_alias)
            rospy.loginfo('zaber_stage_controller initialized!')
            self._home_action.start()
            self._initialized = True

    def _cmd_vel_callback(self,data):
        if self._initialized:
            x_vel = data.linear.x
            y_vel = data.linear.y
            z_vel = data.linear.z
            self._stage.move_x_at_speed(x_vel)
            self._stage.move_y_at_speed(y_vel)
            self._stage.move_z_at_speed(z_vel)

    def _stop_x_callback(self,data):
        if self._initialized:
            self._stage.stop_x()

    def _stop_y_callback(self,data):
        if self._initialized:
            self._stage.stop_y()

    def _stop_z_callback(self,data):
        if self._initialized:
            self._stage.stop_z()

    def _get_pose_callback(self,req):
        while not self._initialized:
            self._rate.sleep()
        pose = Pose()
        positions = self._stage.get_positions()
        pose.position.x = positions[0]
        pose.position.y = positions[1]
        pose.position.z = positions[2]
        return GetPoseResponse(pose)

    def _moving_callback(self,req):
        while not self._initialized:
            self._rate.sleep()
        res = MovingResponse()
        moving = self._stage.moving()
        res.x = moving[0]
        res.y = moving[1]
        res.z = moving[2]
        return res

    def _home_callback(self,req):
        while not self._initialized:
            self._rate.sleep()
        self._stage.home()
        finished = False
        while not finished:
            positions = self._stage.get_positions()
            at_zero = [position == 0 for position in positions]
            finished = all(at_zero)
            if not finished:
                self._rate.sleep()
        self._home_action.set_succeeded()


if __name__ == '__main__':
    try:
        rospy.init_node('zaber_stage_controller')
        zsc = ZaberStageController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
