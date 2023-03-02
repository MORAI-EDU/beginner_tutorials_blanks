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
from class_defs.synced_signal_set import SyncedSignalSet

import numpy as np

from collections import OrderedDict


class IntersectionController(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, id=None):
        self.idx = id      
        self.point = None
        self.synced_signal_id_list = []
        self.synced_signal_set = SyncedSignalSet()


    def get_synced_signal_set(self):
        return synced_signal_set


    def get_intersection_controller_points(self):
        points = []
        for synced_signal_id in self.synced_signal_set.synced_signals:
            points = points + self.synced_signal_set.synced_signals[synced_signal_id].get_synced_signal_points()
        
        return points


    @staticmethod
    def to_dict(obj):
        """json 파일등으로 저장할 수 있는 dict 데이터로 변경한다"""

        dict_data = {
            'idx': obj.idx,
            'synced_signal_id_list': obj.synced_signal_id_list,          
            'point': obj.point.tolist()
        }

        return dict_data


    @staticmethod
    def from_dict(dict_data, synced_tl_set=None):
        """json 파일등으로부터 읽은 dict 데이터에서 IntersectionController 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""
        # 필수 정보
        idx = dict_data['idx']
        point = dict_data['point']

        # 연결된 객체 참조용 정보
        synced_signal_id_list = dict_data['synced_signal_id_list']
        
        """STEP #2 인스턴스 생성"""
        # 필수 정보
        obj = IntersectionController(idx)
        obj.point = np.array(point)

        # 연결된 객체 참조용 정보
        obj.synced_signal_id_list = synced_signal_id_list

        """STEP #3 인스턴스 메소드 호출해서 설정할 값들 설정하기"""       
        if synced_tl_set is not None:
            for synced_signal_id in synced_signal_id_list:
                if synced_signal_id in synced_tl_set.synced_signals.keys():
                    synced_signal = synced_tl_set.synced_signals[synced_signal_id]
                    obj.synced_signal_set.append_synced_signal(synced_signal)

        return obj

    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx }
        prop_data['point'] = {'type' : 'list<float>', 'value' : self.point.tolist()}
        prop_data['synced_signal_id_list'] = {'type' : 'list<string>', 'value' : self.synced_signal_id_list}

        return prop_data