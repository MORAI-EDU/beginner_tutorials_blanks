#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from utils.logger import Logger

import numpy as np
from class_defs.crosswalk import CrossWalk
from class_defs.key_maker import KeyMaker


class CrossWalkSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.data = dict()
        self.key_maker = KeyMaker('CW')

    def append_data(self, cw, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            for idx in self.data.keys():
                idx = self.key_maker.get_new()

            cw.idx = idx

        self.data[cw.idx] = cw

    def remove_data(self, cw):
        self.data.pop(cw.idx)

    def draw_plot(self, axes):
        for idx, cw in self.data.items():
            cw.draw_plot(axes)

    def erase_plot(self):
        for idx, cw in self.data.items():
            cw.erase_plot()