#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function, division
import zaber_device
from exceptions import Exception


class ZaberStageError(Exception):
    def __init__(self,value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class ZaberStage(object):
    '''
    This Python package (zaber_stage) creates a class named
    ZaberStage, which contains an instance of ZaberDevices and adds
    methods to it to use it as an xyz stage.
    Example Usage:
    stage = ZaberStage() # Automatically finds devices if available
    stage.get_aliases()
    {123: [10, 11]}
    stage.set_x_axis(123,10)
    stage.set_y_axis(123,11)
    stage.home()
    stage.moving()
    (True,True,True)
    stage.moving()
    (False,False,False)
    stage.get_positions()
    (0,0,0)
    stage.move_x_at_speed(1000)
    stage.moving()
    (True,False,False)
    stage.get_positions()
    (35898, 0, 0)
    stage.stop_x()
    '''
    def __init__(self,*args,**kwargs):
        self._devs = zaber_device.ZaberDevices()
        self._x_axis = None
        self._y_axis = None
        self._z_axis = None

    def get_aliases(self):
        aliases = {}
        for serial_number in self._devs:
            dev = self._devs[serial_number]
            alias = dev.get_alias()
            aliases[serial_number] = alias
        return aliases

    def set_aliases(self,aliases):
        aliases_prev = self.get_aliases()
        if aliases_prev.keys() != aliases.keys():
            error_string = 'aliases.keys() must equal: {0}'.format(aliases_prev.keys())
            raise ZaberStageError(error_string)
        for serial_number in aliases:
            try:
                if len(aliases_prev[serial_number]) != len(aliases[serial_number]):
                    error_string = 'len(aliases[{0}]) must equal {1}'.format(serial_number,len(aliases_prev[serial_number]))
                    raise ZaberStageError(error_string)
            except TypeError:
                error_string = 'aliases[{0}] is incorrect type'.format(serial_number)
                raise ZaberStageError(error_string)
            dev = self._devs[serial_number]
            for actuator in range(len(aliases[serial_number])):
                dev.set_alias(actuator,aliases[serial_number][actuator])

    def _set_axis(self,axis,serial_number,alias):
        ax = {}
        ax['serial_number'] = serial_number
        ax['dev'] = self._devs[serial_number]
        ax['alias'] = alias
        aliases = self.get_aliases()
        ax['actuator'] = aliases[serial_number].index(alias)
        if axis == 'x':
            self._x_axis = ax
        elif axis == 'y':
            self._y_axis = ax
        elif axis == 'z':
            self._z_axis = ax

    def set_x_axis(self,serial_number,alias):
        self._set_axis('x',serial_number,alias)

    def set_y_axis(self,serial_number,alias):
        self._set_axis('y',serial_number,alias)

    def set_z_axis(self,serial_number,alias):
        self._set_axis('z',serial_number,alias)

    def _move_at_speed(self,axis,speed):
        if axis == 'x':
            ax = self._x_axis
        elif axis == 'y':
            ax = self._y_axis
        elif axis == 'z':
            ax = self._z_axis
        speed = int(speed)
        if ax is not None:
            dev = ax['dev']
            alias = ax['alias']
            dev.move_at_speed(speed,alias)

    def move_x_at_speed(self,speed):
        self._move_at_speed('x',speed)

    def move_y_at_speed(self,speed):
        self._move_at_speed('y',speed)

    def move_z_at_speed(self,speed):
        self._move_at_speed('z',speed)

    def _stop(self,axis):
        if axis == 'x':
            ax = self._x_axis
        elif axis == 'y':
            ax = self._y_axis
        elif axis == 'z':
            ax = self._z_axis
        if ax is not None:
            dev = ax['dev']
            alias = ax['alias']
            dev.stop(alias)

    def stop_x(self):
        self._stop('x')

    def stop_y(self):
        self._stop('y')

    def stop_z(self):
        self._stop('z')

    def get_positions(self):
        positions = {}
        for serial_number in self._devs:
            dev = self._devs[serial_number]
            position = dev.get_position()
            positions[serial_number] = position
        if self._x_axis is not None:
            x_position = positions[serial_number][self._x_axis['actuator']]
        else:
            x_position = 0
        if self._y_axis is not None:
            y_position = positions[serial_number][self._y_axis['actuator']]
        else:
            y_position = 0
        if self._z_axis is not None:
            z_position = positions[serial_number][self._z_axis['actuator']]
        else:
            z_position = 0
        return x_position,y_position,z_position

    def moving(self):
        movings = {}
        for serial_number in self._devs:
            dev = self._devs[serial_number]
            moving = dev.moving()
            movings[serial_number] = moving
        if self._x_axis is not None:
            x_moving = movings[serial_number][self._x_axis['actuator']]
        else:
            x_moving = False
        if self._y_axis is not None:
            y_moving = movings[serial_number][self._y_axis['actuator']]
        else:
            y_moving = False
        if self._z_axis is not None:
            z_moving = movings[serial_number][self._z_axis['actuator']]
        else:
            z_moving = False
        return x_moving,y_moving,z_moving

    def home(self):
        for serial_number in self._devs:
            dev = self._devs[serial_number]
            dev.home()

    def stop(self):
        for serial_number in self._devs:
            dev = self._devs[serial_number]
            dev.stop()
