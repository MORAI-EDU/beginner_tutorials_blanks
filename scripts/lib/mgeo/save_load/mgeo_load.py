#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, '../'))) # mgeo가 있는 경로를 추가한다.

from class_defs import *

import numpy as np
import matplotlib.pyplot as plt
import json


def temp_find_node_by_id(node_set, idx):
    for node in node_set.nodes:
        if node.idx == idx:
            return node
    raise BaseException('[ERROR] cannot find temp_find_node_by_id!')
        

def load(file_path, random_search=False):
    '''
    '''
    filename = os.path.join(file_path, 'node_set.json')
    with open(filename, 'r') as f:
        node_save_info_list = json.load(f)

    filename = os.path.join(file_path, 'line_set.json')
    with open(filename, 'r') as f:
        line_save_info_list = json.load(f)

    filename = os.path.join(file_path, 'plane_set.json')
    with open(filename, 'r') as f:
        plane_save_info_list = json.load(f)

    # node_save_info_list에서 node_set 생성
    node_set = NodeSet()
    line_set = LineSet()
    plane_set = PlaneSet()

    for save_info in node_save_info_list:
        idx = save_info['idx']
        point = save_info['point']
        node_type = save_info['node_type']
        
        node = Node(idx)
        node.point = np.array(point)
        node_set.nodes.append(node)

    # line_save_info_list에서 line_set 생성
    for save_info in line_save_info_list:
        idx = save_info['idx']
        from_node_idx = save_info['from_node_idx']
        to_node_idx = save_info['to_node_idx']
        points = save_info['points']


        # random_search는 언제 사용하는가?
        # 기존의 코드에서 (또는 기존에 수정하여 저장한 mgeo 파일에서)
        # node의 idx를 이용하여 node_set.nodes[idx]로 접근이 불가능한 경우이다.
        # if node_set.nodes.idx == idx 를 일일이 검색해야하는 상황
        if random_search:
            # 가정: node_set.nodes 에 저장된 각 node의 idx값은 
            # node_set.nodes 리스트에서의 위치와 관계가 없는 상태이다.

            line = Line(np.array(points), idx)

            # to_node, from_node를 전체 node_set을 검색하여 해결한다.
            for node in node_set.nodes:
                if node.idx == from_node_idx:
                    line.set_from_node(node)
                if node.idx == to_node_idx:
                    line.set_to_node(node)

            line_set.lines.append(line) 
        else:
            # 가정: node_set.nodes 에 저장된 각 node의 idx값은 
            # node_set.nodes 리스트에서의 위치와 같도록 정리된 상태이다.

            # TEMP: FOR DEBUGGING
            if from_node_idx > len(node_set.nodes):
                for i in range(len(node_set.nodes)):
                    node = node_set.nodes[i]
                    if from_node_idx == node.idx:
                        print('i = {}, from_node_idx = {}'.format(i, from_node_idx))
                raise BaseException('[ERROR] from_node_idx = {} > node_set.nodes = {}'.format(from_node_idx, len(node_set.nodes)))
            if to_node_idx > len(node_set.nodes):
                raise BaseException('[ERROR] to_node_idx   = {} > node_set.nodes = {}'.format(to_node_idx, len(node_set.nodes)))
            # End of TEMP

            line = Line(np.array(points), idx)
            line.set_from_node(node_set.nodes[from_node_idx])
            line.set_to_node(node_set.nodes[to_node_idx])
            line_set.lines.append(line) 

    # plane_save_info_list에서 plane_set 생성
    for save_info in plane_save_info_list:
        # idx = save_info['idx']
        node_idx_list = save_info['node_idx_list']

        # index를 정리하면서 load한다
        idx = len(plane_set.planes)
        plane = Plane(idx)

        # NOTE: 원래는 이 코드를 쓰고 싶었는데, 
        # 이 코드에서는 내부적으로 node id로 node_set에서 node를 찾을 때
        # 개별 node의 idx가 nodes에서의 위치를 나타내는 줄 알고 사용헀다
        # 그런데, 현재 mgeo 파일에 저장된 데이터는 그렇지 않아서 동작이 안 됨
        # plane.init_from_node_idx_list(node_set, node_idx_list)

        # 대신 아래 코드를 이용한다
        # 그런데 여기서 또 문제가 있다.
        # Node ID에 겹치는 것이 있다는 것..
        for node_idx in node_idx_list:
            # node = temp_find_node_by_id(node_set, node_idx)

            for node in node_set.nodes:
                if node.idx == node_idx:
                    plane.append_node(node)

        plane_set.planes.append(plane)

    # line_set에 plane 정보 입력  
    for plane in plane_set.planes:
        for line in plane.line_connection:
            line['line'].add_included_plane(plane)

    return node_set, line_set, plane_set


def load_node_and_link(file_path):
    '''
    '''
    filename = os.path.join(file_path, 'node_set.json')
    with open(filename, 'r') as f:
        node_save_info_list = json.load(f)

    filename = os.path.join(file_path, 'link_set.json')
    with open(filename, 'r') as f:
        line_save_info_list = json.load(f)

    # 버전 정보를 찾는다
    # 버전 파일이 없으면, ver1이다.
    filename = os.path.join(file_path, 'data_format_info.json')
    if not os.path.isfile(filename):
        print('[WARNING] There is no data_format_info.json file in the specified location. link format ver1 is assumed.')

        import subproc_load_link_ver1
        return subproc_load_link_ver1.load_node_and_link(node_save_info_list, line_save_info_list)
    
    with open(filename, 'r') as f:
        data_format_info = json.load(f)

    if data_format_info['maj_ver'] == 2:
        import subproc_load_link_ver2
        return subproc_load_link_ver2.load_node_and_link(node_save_info_list, line_save_info_list, data_format_info)

    print('[INFO] Ended')
    
    
    



