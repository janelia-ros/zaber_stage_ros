#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function, division
import zaber_device


class ZaberStage(object):
    def __init__(self,*args,**kwargs):
        self._devs = zaber_device.ZaberDevices()
        self.x_axis = None
        self.y_axis = None
        self.z_axis = None

    def get_aliases(self):
        aliases = {}
        for serial_number in self._devs:
            dev = self._devs[serial_number]
            alias = dev.get_alias()
            aliases[serial_number] = alias
        return aliases

    def home(self):
        for serial_number in self._devs:
            dev = self._devs[serial_number]
            dev.home()
