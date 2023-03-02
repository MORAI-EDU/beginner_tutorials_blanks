#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from utils.logger import Logger

import matplotlib.pyplot as plt
import numpy as np 
from class_defs.surface_marking import SurfaceMarking


class CrossWalk(SurfaceMarking):
    def __init__(self, points=None, idx=None):
        super(CrossWalk, self).__init__(points, idx)