#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from utils.logger import Logger

from class_defs.base_point import BasePoint
from class_defs.signal import Signal
from class_defs.signal_set import SignalSet
from class_defs.synced_signal import SyncedSignal
from class_defs.intersection_controller import IntersectionController
from class_defs.key_maker import KeyMaker

import numpy as np


class IntersectionControllerSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.intersection_controllers = dict()
        self.key_maker = KeyMaker('IC')


    def append_synced_signal(self, ic_obj, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            for idx in self.intersection_controllers.keys():
                idx = self.key_maker.get_new()

            ic_obj.idx = idx

        self.intersection_controllers[ic_obj.idx] = ic_obj