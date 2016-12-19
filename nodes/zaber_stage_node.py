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
from zaber_stage.srv import GetPoseAndDebugInfo,GetPoseAndDebugInfoResponse

from zaber_stage.msg import EmptyAction
from zaber_stage.msg import MoveAction


class ZaberStageController(object):
    def __init__(self,*args,**kwargs):
        rospy.loginfo('Initializing zaber_stage_node...')
        self._initialized = False
        self._rate = rospy.Rate(4)

        self._cmd_vel_sub = rospy.Subscriber('~cmd_vel',Twist,self._cmd_vel_callback)
        self._stop_x_sub = rospy.Subscriber('~stop_x',Empty,self._stop_x_callback)
        self._stop_y_sub = rospy.Subscriber('~stop_y',Empty,self._stop_y_callback)
        self._stop_z_sub = rospy.Subscriber('~stop_z',Empty,self._stop_z_callback)
        self._stop_sub = rospy.Subscriber('~stop',Empty,self._stop_callback)
        self._get_pose_srv = rospy.Service('~get_pose',GetPose,self._get_pose_callback)
        self._moving_srv = rospy.Service('~moving',Moving,self._moving_callback)
        self._get_pose_and_debug_info_srv = rospy.Service('~get_pose_and_debug_info',GetPoseAndDebugInfo,self._get_pose_and_debug_info_callback)
        self._home_action = actionlib.SimpleActionServer('~home', EmptyAction, self._home_callback, False)
        self._move_relative_action = actionlib.SimpleActionServer('~move_relative', MoveAction, self._move_relative_callback, False)
        self._move_absolute_action = actionlib.SimpleActionServer('~move_absolute', MoveAction, self._move_absolute_callback, False)
        self._move_relative_percent_action = actionlib.SimpleActionServer('~move_relative_percent', MoveAction, self._move_relative_percent_callback, False)
        self._move_absolute_percent_action = actionlib.SimpleActionServer('~move_absolute_percent', MoveAction, self._move_absolute_percent_callback, False)

        x_serial_number = rospy.get_param('~x_serial_number', None)
        if x_serial_number == '':
            x_serial_number = None
        x_alias = rospy.get_param('~x_alias', None)
        if x_alias == '':
            x_alias = None
        x_microstep_size = rospy.get_param('~x_microstep_size', 1)
        x_travel = rospy.get_param('~x_travel', None)
        if x_travel == '':
            x_travel = None
        y_serial_number = rospy.get_param('~y_serial_number', None)
        if y_serial_number == '':
            y_serial_number = None
        y_alias = rospy.get_param('~y_alias', None)
        if y_alias == '':
            y_alias = None
        y_microstep_size = rospy.get_param('~y_microstep_size', 1)
        y_travel = rospy.get_param('~y_travel', None)
        if y_travel == '':
            y_travel = None
        z_serial_number = rospy.get_param('~z_serial_number', None)
        if z_serial_number == '':
            z_serial_number = None
        z_alias = rospy.get_param('~z_alias', None)
        if z_alias == '':
            z_alias = None
        z_microstep_size = rospy.get_param('~z_microstep_size', 1)
        z_travel = rospy.get_param('~z_travel', None)
        if z_travel == '':
            z_travel = None

        axis_set = False
        if (x_serial_number is not None) and (x_alias is not None):
            axis_set = True
        if (y_serial_number is not None) and (y_alias is not None):
            axis_set = True
        if (z_serial_number is not None) and (z_alias is not None):
            axis_set = True
        if not axis_set:
            err_str = "Not enough zaber_stage_node axis arguments specified! (x_serial_number,x_alias,y_serial_number,y_alias,z_serial_number,z_alias)"
            rospy.signal_shutdown(err_str)
            rospy.logerr(err_str)
        else:
            serial_port = rospy.get_param('~serial_port', None)
            if serial_port is not None:
                self._stage = ZaberStage(port=serial_port)
            else:
                self._stage = ZaberStage()
            if (x_serial_number is not None) and (x_alias is not None):
                self._stage.set_x_axis(x_serial_number,x_alias)
            if (y_serial_number is not None) and (y_alias is not None):
                self._stage.set_y_axis(y_serial_number,y_alias)
            if (z_serial_number is not None) and (z_alias is not None):
                self._stage.set_z_axis(z_serial_number,z_alias)
            self._stage.set_x_microstep_size(x_microstep_size)
            self._stage.set_y_microstep_size(y_microstep_size)
            self._stage.set_z_microstep_size(z_microstep_size)
            if x_travel is not None:
                self._stage.set_x_travel(x_travel)
            if y_travel is not None:
                self._stage.set_y_travel(y_travel)
            if z_travel is not None:
                self._stage.set_z_travel(z_travel)
            rospy.loginfo('zaber_stage_node initialized!')
            self._home_action.start()
            self._move_relative_action.start()
            self._move_absolute_action.start()
            self._move_relative_percent_action.start()
            self._move_absolute_percent_action.start()
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

    def _stop_callback(self,data):
        if self._initialized:
            self.stop()

    def stop(self):
        if self._initialized:
            self._stage.stop_x()
            self._stage.stop_y()
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

    def _get_pose_and_debug_info_callback(self,req):
        while not self._initialized:
            self._rate.sleep()
        res = GetPoseAndDebugInfoResponse()
        positions_and_debug = self._stage.get_positions_and_debug_info()
        res.pose_and_debug_info.pose.position.x = positions_and_debug['position'][0]
        res.pose_and_debug_info.pose.position.y = positions_and_debug['position'][1]
        res.pose_and_debug_info.pose.position.z = positions_and_debug['position'][2]
        res.pose_and_debug_info.pose_microstep.position.x = positions_and_debug['position_microstep'][0]
        res.pose_and_debug_info.pose_microstep.position.y = positions_and_debug['position_microstep'][1]
        res.pose_and_debug_info.pose_microstep.position.z = positions_and_debug['position_microstep'][2]
        res.pose_and_debug_info.zaber_response = positions_and_debug['response']
        return res

    def _home_callback(self,req):
        while not self._initialized:
            self._rate.sleep()
        self._stage.home()
        rospy.sleep(2)
        finished = False
        while not finished:
            homed = self._stage.homed()
            print(homed)
            finished = all(homed)
            if not finished:
                self._rate.sleep()
        self._home_action.set_succeeded()

    def _move_relative_callback(self,req):
        while not self._initialized:
            self._rate.sleep()
        rospy.loginfo("move_relative: x={0}, y={1}, z={2}".format(req.pose.position.x,
                                                                  req.pose.position.y,
                                                                  req.pose.position.z))
        self._stage.move_x_relative(req.pose.position.x)
        self._stage.move_y_relative(req.pose.position.y)
        self._stage.move_z_relative(req.pose.position.z)
        finished = False
        while not finished:
            moving = self._stage.moving()
            finished = not any(moving)
            if not finished:
                self._rate.sleep()
        self._move_relative_action.set_succeeded()

    def _move_absolute_callback(self,req):
        while not self._initialized:
            self._rate.sleep()
        rospy.loginfo("move_absolute: x={0}, y={1}, z={2}".format(req.pose.position.x,
                                                                  req.pose.position.y,
                                                                  req.pose.position.z))
        self._stage.move_x_absolute(req.pose.position.x)
        self._stage.move_y_absolute(req.pose.position.y)
        self._stage.move_z_absolute(req.pose.position.z)
        finished = False
        while not finished:
            moving = self._stage.moving()
            finished = not any(moving)
            if not finished:
                self._rate.sleep()
        self._move_absolute_action.set_succeeded()

    def _move_relative_percent_callback(self,req):
        while not self._initialized:
            self._rate.sleep()
        rospy.loginfo("move_relative_percent: x={0}%, y={1}%, z={2}%".format(req.pose.position.x,
                                                                             req.pose.position.y,
                                                                             req.pose.position.z))
        self._stage.move_x_relative_percent(req.pose.position.x)
        self._stage.move_y_relative_percent(req.pose.position.y)
        self._stage.move_z_relative_percent(req.pose.position.z)
        finished = False
        while not finished:
            moving = self._stage.moving()
            finished = not any(moving)
            if not finished:
                self._rate.sleep()
        self._move_relative_percent_action.set_succeeded()

    def _move_absolute_percent_callback(self,req):
        while not self._initialized:
            self._rate.sleep()
        rospy.loginfo("move_absolute_percent: x={0}%, y={1}%, z={2}%".format(req.pose.position.x,
                                                                             req.pose.position.y,
                                                                             req.pose.position.z))
        self._stage.move_x_absolute_percent(req.pose.position.x)
        self._stage.move_y_absolute_percent(req.pose.position.y)
        self._stage.move_z_absolute_percent(req.pose.position.z)
        finished = False
        while not finished:
            moving = self._stage.moving()
            finished = not any(moving)
            if not finished:
                self._rate.sleep()
        self._move_absolute_percent_action.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('zaber_stage_node')
    zsc = ZaberStageController()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        zsc.stop()
