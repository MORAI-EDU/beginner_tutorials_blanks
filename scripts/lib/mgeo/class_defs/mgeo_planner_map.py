#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from utils.logger import Logger

import numpy as np
import matplotlib.pyplot as plt
import json

# 이 support.py 만 릴리즈 시 변경
from class_defs.support import supported_class

from class_defs.node import Node
from class_defs.link import Link
from class_defs.signal import Signal
from class_defs.lane_marking import LaneMarking
from class_defs.junction import Junction
from class_defs.junction_set import JunctionSet
from class_defs.node_set import NodeSet
from class_defs.line_set import LineSet
from class_defs.lane_marking_set import LaneMarkingSet

from class_defs.signal_set import SignalSet

if supported_class['synced_signal']:
    from class_defs.synced_signal import SyncedSignal
    from class_defs.synced_signal_set import SyncedSignalSet

if supported_class['intersection_controller']:
    from class_defs.intersection_controller import IntersectionController
    from class_defs.intersection_controller_set import IntersectionControllerSet

from class_defs.surface_marking import SurfaceMarking
from class_defs.surface_marking_set import SurfaceMarkingSet

from utils.version import Version


class MGeoPlannerMap(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, 
        node_set=NodeSet(), link_set=LineSet(), lane_marking_set = LaneMarkingSet(), lane_node_set=NodeSet(), junction_set=JunctionSet(),
        sign_set=SignalSet(), light_set=SignalSet(), synced_light_set=SyncedSignalSet(), intersection_controller_set=IntersectionControllerSet(), sm_set=SurfaceMarkingSet()):
        '''
        반드시 MGeoPlannerMap은 node_set, link_set을 가지고 있어야 함
        Ctor에 전달하면서 init한다

        ver2.1 -> ver2.2 update: link 출력에 max_speed 추가 
        ver2.2 -> ver2.3 update: junction 클래스 추가
        ver2.3 -> ver2.4 update: code42 지도 데이터 추가
        ver2.4 -> ver2.5 update: junction을 list 형태로 변경
        ver2.5 -> ver2.6 update: 선의 fitting 방식을 달리하도록 데이터 필드 추가
        ver2.6 -> ver2.7 update: surface marking set 추가
        ver2.7 -> ver2.8 update: link의 related signal 정의 변경
        '''
        # geometry
        self.node_set = node_set
        self.link_set = link_set

        self.junction_set = junction_set
    
        self.sign_set = sign_set
        self.light_set = light_set

        self.synced_light_set = synced_light_set
        self.intersection_controller_set = intersection_controller_set
        self.lane_marking_set = lane_marking_set
        self.lane_node_set = lane_node_set

        self.sm_set = sm_set

        # 기타 정보
        self.maj_ver = 2
        self.min_ver = 8
        self.global_coordinate_system = 'UTM52N'
        self.local_origin_in_global = np.array([0, 0, 0])


    def set_coordinate_system_from_prj_file(self, prj_file):
        """SHP 파일 등에 포함되는 .prj 파일을 읽고 표준 proj4 string 포맷의 값으로 변환 & 저장한다.
        GDAL package를 필요로 한다. 
        """
        self.global_coordinate_system = MGeoPlannerMap.esri_prj_to_proj4_string(prj_file)


    def set_origin(self, origin):
        if isinstance(origin, np.ndarray):
            self.local_origin_in_global = origin
        else:
            self.local_origin_in_global = np.array(origin)


    def get_origin(self):
        return self.local_origin_in_global


    def to_json(self, output_path):
        MGeoPlannerMap.save_global_info(output_path, self)
        MGeoPlannerMap.save_node(output_path, self.node_set)
        MGeoPlannerMap.save_link(output_path, self.link_set)
        
        if self.sign_set is not None:
            MGeoPlannerMap.save_traffic_sign(output_path, self.sign_set)
        
        if self.light_set is not None:
            MGeoPlannerMap.save_traffic_light(output_path, self.light_set)

        if self.synced_light_set is not None:
            MGeoPlannerMap.save_synced_traffic_light(output_path, self.synced_light_set)

        if self.intersection_controller_set is not None:
            MGeoPlannerMap.save_intersection_controller(output_path, self.intersection_controller_set)

        if self.sm_set is not None:
            MGeoPlannerMap.save_surface_marking(output_path, self.sm_set)

        if self.lane_marking_set is not None:
            MGeoPlannerMap.save_lane_marking(output_path, self.lane_marking_set)

        if self.lane_node_set is not None:
            MGeoPlannerMap.save_lane_node(output_path, self.lane_node_set)

            

    @staticmethod
    def esri_prj_to_proj4_string(prj_file):
        """SHP 파일 등에 포함되는 .prj 파일을 읽고 표준 proj4 string 포맷의 값으로 변환한다.
        GDAL package를 필요로 한다. 
        """
        from osgeo import osr

        prj_file = open(prj_file, 'r')
        prj_txt = prj_file.read()
        srs = osr.SpatialReference()
        srs.ImportFromESRI([prj_txt])
        
        Proj4 = srs.ExportToProj4()
        return Proj4


    @staticmethod
    def save_global_info(output_path, obj):
        # 클래스 내 필드에서 global_info 설정
        global_info = {
            'maj_ver': obj.maj_ver,
            'min_ver': obj.min_ver,
            'global_coordinate_system': obj.global_coordinate_system, # 
            'local_origin_in_global': obj.local_origin_in_global.tolist(), # origin
            'lane_change_link_included': True, 
            'license':'MORAI Inc.'
        }
        filename = os.path.join(output_path, 'global_info.json')
        with open(filename, 'w') as f:
            json.dump(global_info, f, indent=2)


    @staticmethod
    def save_node(output_path, node_set):
        # 각 노드는 id와 point를 저장한다.
        save_info_list = []
        for var, node in node_set.nodes.items():
            dict_data = node.to_dict()
            save_info_list.append(dict_data)

        # 이를 저장한다.
        filename = os.path.join(output_path, 'node_set.json')
        with open(filename, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_link(output_path, link_set):
        save_info_list = []
        for idx, line in link_set.lines.items():
            dict_data = line.to_dict()
            save_info_list.append(dict_data)
            
        filename = os.path.join(output_path, 'link_set.json')
        with open(filename, 'w') as f:
            json.dump(save_info_list, f, indent=2)        


    @staticmethod
    def save_traffic_light(output_path, light_set): 
        save_info_list = []
        for var, tl in light_set.signals.items():
            dict_data = Signal.to_dict(tl)
            save_info_list.append(dict_data)
      
        file_path = os.path.join(output_path, 'traffic_light_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_synced_traffic_light(output_path, synced_light_set): 
        save_info_list = []
        for var, synced_tl in synced_light_set.synced_signals.items():
            dict_data = SyncedSignal.to_dict(synced_tl)
            save_info_list.append(dict_data)
      
        file_path = os.path.join(output_path, 'synced_traffic_light_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_intersection_controller(output_path, intersection_controller_set): 
        save_info_list = []
        for var, ic in intersection_controller_set.intersection_controllers.items():
            dict_data = IntersectionController.to_dict(ic)
            save_info_list.append(dict_data)
      
        file_path = os.path.join(output_path, 'intersection_controller_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_traffic_sign(output_path, sign_set):
        save_info_list = []
        for var, ts in sign_set.signals.items():           
            dict_data = Signal.to_dict(ts)
            save_info_list.append(dict_data)

        # json 파일로 저장
        file_path = os.path.join(output_path, 'traffic_sign_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_surface_marking(output_path, sm_set):
        save_info_list = []
        for key, sm in sm_set.data.items():
            dict_data = SurfaceMarking.to_dict(sm)
            save_info_list.append(dict_data)

        # json 파일로 저장
        file_path = os.path.join(output_path, 'surface_marking_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)

    @staticmethod
    def save_lane_marking(output_path, lane_marking_set):
        save_info_list = []
        for idx, lane in lane_marking_set.lanes.items():
            dict_data = lane.to_dict()
            save_info_list.append(dict_data)
            
        filename = os.path.join(output_path, 'lane_marking_set.json')
        with open(filename, 'w') as f:
            json.dump(save_info_list, f, indent=2)     
        
    @staticmethod
    def save_lane_node(output_path, lane_node):
        # 각 노드는 id와 point를 저장한다.
        save_info_list = []
        for var, node in lane_node.nodes.items():
            dict_data = node.to_dict()
            save_info_list.append(dict_data)

        # 이를 저장한다.
        filename = os.path.join(output_path, 'lane_node_set.json')
        with open(filename, 'w') as f:
            json.dump(save_info_list, f, indent=2)

    @staticmethod
    def load_node_and_link(folder_path):
        '''
        파일을 읽어 global_info, node_set, link_set을 생성하여 리턴한다
        MGeoPlannerMap ver2.1 까지 지원
        '''
        # node_set, link_set은 버전 정보와 관계없이 공통이다.
        filename = os.path.join(folder_path, 'node_set.json')
        with open(filename, 'r') as f:
            node_save_info_list = json.load(f)

        filename = os.path.join(folder_path, 'link_set.json')
        with open(filename, 'r') as f:
            line_save_info_list = json.load(f)


        # 버전 정보를 찾는다
        # 버전 파일이 없으면, ver1이다.
        # global_info.json 대신 global_info.mprj 파일이 있으면 mprj 파일로 읽어오기
        filename = os.path.join(folder_path, 'global_info.json')
        if os.path.exists(os.path.join(folder_path, 'global_info.mprj')):
            filename = os.path.join(folder_path, 'global_info.mprj')
        if not os.path.isfile(filename):
            Logger.log_warning('There is no global_info.json file in the specified location. link format ver1 is assumed.')

            from save_load import subproc_load_link_ver1
            node_set, link_set = subproc_load_link_ver1.load_node_and_link(node_save_info_list, line_save_info_list)

            # ver1에서는 global_info가 없으므로, 직접 생성해준다
            global_info = {
                'maj_ver': 1,
                'min_ver': 0,
                'global_coordinate_system': 'UTM52N',
                'local_origin_in_global': [0, 0, 0]
            }

            return global_info, node_set, link_set

        # 읽을 버전 정보 파일이 있는 경우    
        with open(filename, 'r') as f:
            global_info = json.load(f)

        # 버전 정보에 맞게 node_set, link_set을 읽어온다.
        if global_info['maj_ver'] == 2:

            from save_load import subproc_load_link_ver2
            node_set, link_set, junction_set = subproc_load_link_ver2.load_node_and_link(
                node_save_info_list, line_save_info_list, global_info)

        return global_info, node_set, link_set, junction_set


    @staticmethod
    def load_traffic_sign(folder_path, link_set):
        """traffic_sign_set.json 파일을 읽고 표지판 셋 (ts_set)을 생성한다"""
        ts_set = SignalSet()
        filename = os.path.join(folder_path, 'traffic_sign_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            Logger.log_warning('There is no traffic_sign_set.json. An empty SignalSet instance will be returned.')
            return ts_set

        for each_info in saved_info:
            ts = Signal.from_dict(each_info, link_set)
            ts_set.append_signal(ts)

        return ts_set


    @staticmethod
    def load_traffic_light(folder_path, link_set):
        """traffic_light_set.json 파일을 읽고 표지판 셋 (tl_set)을 생성한다"""
        tl_set = SignalSet()
        filename = os.path.join(folder_path, 'traffic_light_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            Logger.log_warning('There is no traffic_light_set.json. An empty SignalSet instance will be returned.')
            return tl_set

        for each_info in saved_info:
            tl = Signal.from_dict(each_info, link_set)
            tl_set.append_signal(tl)

        return tl_set


    @staticmethod
    def load_synced_traffic_light(folder_path, link_set, tl_set):
        """synced_traffic_light_set.json 파일을 읽고 synced_tl_set을 생성한다"""
        synced_tl_set = SyncedSignalSet()
        filename = os.path.join(folder_path, 'synced_traffic_light_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            Logger.log_warning('There is no synced_traffic_light_set.json. An empty SyncedSignalSet instance will be returned.')
            return synced_tl_set

        for each_info in saved_info:
            synced_tl = SyncedSignal.from_dict(each_info, link_set, tl_set)
            synced_tl_set.append_synced_signal(synced_tl)

        return synced_tl_set


    @staticmethod
    def load_intersection_controller(folder_path, synced_tl_set):
        """synced_traffic_light_set.json 파일을 읽고 synced_tl_set을 생성한다"""
        ic_set = IntersectionControllerSet()
        filename = os.path.join(folder_path, 'intersection_controller_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            Logger.log_warning('There is no intersection_controller_set.json. An empty IntersectionControllerSet instance will be returned.')
            return ic_set

        for each_info in saved_info:
            ic = IntersectionController.from_dict(each_info, synced_tl_set)
            ic_set.append_synced_signal(ic)

        return ic_set


    @staticmethod
    def load_surface_marking(folder_path, link_set):
        """surface_marking_set.json 파일을 읽고 surface_marking셋 (sm_set)을 생성한다"""
        sm_set = SurfaceMarkingSet()
        filename = os.path.join(folder_path, 'surface_marking_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            Logger.log_warning('There is no traffic_light_set.json. An empty SignalSet instance will be returned.')
            return sm_set

        for each_info in saved_info:
            sm = SurfaceMarking.from_dict(each_info, link_set)
            sm_set.append_data(sm)

        return sm_set
        

    @staticmethod
    def load_lane_marking(folder_path):
        """lane_marking_set.json 파일을 읽고 lane_marking셋 (lane_set)을 생성한다"""

        node_set = NodeSet()
        lane_set = LaneMarkingSet()

        filename = os.path.join(folder_path, 'lane_node_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                node_save_info_list = json.load(f)
        else:
            Logger.log_warning('There is no traffic_light_set.json. An empty SignalSet instance will be returned.')
            return lane_set, node_set

        for save_info in node_save_info_list:
            idx = save_info['idx']
            point = save_info['point']
        
            node = Node(idx)
            node.point = np.array(point)

            node_set.append_node(node, create_new_key=False)
    
        filename = os.path.join(folder_path, 'lane_marking_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            Logger.log_warning('There is no lane_marking_set.json. An empty SignalSet instance will be returned.')
            return lane_set, node_set

        for each_info in saved_info:
            lane = LaneMarking.from_dict(each_info, node_set)
            lane_set.append_line(lane)

        return lane_set, node_set

    @staticmethod
    def create_instance_from_json(folder_path):
        '''
        파일을 읽어서 MGeoPlannerMap 인스턴스를 생성한다
        '''
        # global_info.mprj 파일 선택할 수 있게 설정
        if not os.path.isdir(folder_path):
            folder_path = os.path.dirname(folder_path)

        global_info, node_set, link_set, junction_set = MGeoPlannerMap.load_node_and_link(folder_path)

        lane_marking_set, lane_node_set = MGeoPlannerMap.load_lane_marking(folder_path)
         
        sign_set = MGeoPlannerMap.load_traffic_sign(folder_path, link_set)
        light_set = MGeoPlannerMap.load_traffic_light(folder_path, link_set)
        
        if supported_class['synced_signal']:
            synced_light_set = MGeoPlannerMap.load_synced_traffic_light(folder_path, link_set, light_set)
        else:
            synced_light_set = None

        if supported_class['intersection_controller']:
            intersection_controller_set = MGeoPlannerMap.load_intersection_controller(folder_path, synced_light_set)
        else:
            intersection_controller_set = None

        sm_set = MGeoPlannerMap.load_surface_marking(folder_path, link_set)
        
        
        mgeo_planner_map = MGeoPlannerMap(node_set, link_set, lane_marking_set, lane_node_set, junction_set, 
            sign_set, light_set, synced_light_set, intersection_controller_set, sm_set)

        mgeo_planner_map.maj_ver = global_info['maj_ver']
        mgeo_planner_map.min_ver = global_info['min_ver']
        mgeo_planner_map.global_coordinate_system = global_info['global_coordinate_system']
        mgeo_planner_map.local_origin_in_global = np.array(global_info['local_origin_in_global'])

        return mgeo_planner_map
