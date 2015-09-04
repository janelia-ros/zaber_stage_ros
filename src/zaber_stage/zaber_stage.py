#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function, division
import zaber_device


class ZaberStage(object):
    def __init__(self,*args,**kwargs):
        self._devs = zaber_device.ZaberDevices()

    def get_info(self):
        for dev in self._devs:
            serial_number = dev.get_serial_number()
            alias = dev.get_alias()
            print('serial_number: {0}, alias: {1}'.format(serial_number,alias))
