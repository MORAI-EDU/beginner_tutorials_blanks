#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from utils.logger import Logger

import numpy as np

class BasePlane(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, _points=None, idx=None):
        self.points = None
        self.idx = idx

        self.bbox_x = None
        self.bbox_y = None
        self.bbox_z = None

        # 주의: 내부적으로 bbox_x,y,z를 설정하므로,
        # 반드시 self.bbox_x = None 파트 다음에 호출되어야 함
        self.set_points(_points)


    def set_points(self, _points):
        if _points is None:
            return
            
        if type(_points) is np.ndarray:
            self.points = _points
        elif type(_points) is list:
            self.points = np.array(_points)
        else:
            raise BaseException('[ERROR] @ BasePlane.set_points: _points must be an instance of numpy.ndarray of list. Type of your input = {}'.format(type(_points)))
        
        x = _points[:,0]
        y = _points[:,1]
        z = _points[:,2]
        self.set_bbox(x.min(), x.max(), y.min(), y.max(), z.min(), z.max())


    def set_bbox(self, xmin, xmax, ymin, ymax, zmin, zmax):
        self.bbox_x = [xmin, xmax]
        self.bbox_y = [ymin, ymax]
        self.bbox_z = [zmin, zmax]


    def is_out_of_xy_range(self, xlim, ylim):
        """line이 완전히 벗어났을 때만 True. 즉, 살짝 겹쳤을 때는 False이다."""
        if self.bbox_x is None or self.bbox_y is None:
            raise BaseException('[ERROR] bbox is not set')

        x_min = self.bbox_x[0]
        x_max = self.bbox_x[1]
        y_min = self.bbox_y[0]
        y_max = self.bbox_y[1]

        # x축에 대해
        if x_max < xlim[0] or xlim[1] < x_min:
            x_out = True
        else:
            x_out = False

        # y축에 대해
        if y_max < ylim[0] or ylim[1] < y_min:
            y_out = True
        else:
            y_out = False
        
        # 둘 중 하나라도 위와 같이 벗어나면
        # xy range인 box에는 전혀 겹치지 않는다
        return x_out or y_out 
    

    def is_completely_included_in_xy_range(self, xlim, ylim):
        """line이 완전히 포함될 때만 True. 즉, 살짝 겹쳤을 때는 False이다."""

        if self.bbox_x is None or self.bbox_y is None:
            raise BaseException('[ERROR] bbox is not set')

        x_min = self.bbox_x[0]
        x_max = self.bbox_x[1]
        y_min = self.bbox_y[0]
        y_max = self.bbox_y[1]

        if xlim[0] <= x_min and x_max <= xlim[1]:
            x_in = True
        else:
            x_in = False

        if ylim[0] <= y_min and y_max <= ylim[1]:
            y_in = True
        else:
            y_in = False

        # 둘 다 True일 때만 이 xlim, ylim으로 지정된 범위에
        # 완전히 포함된다
        return x_in and y_in