#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, '../'))) # mgeo가 있는 경로를 추가한다.

from class_defs import *
from utils import * 

import numpy as np
import matplotlib.pyplot as plt
import json


def save(output_path, node_set, line_set, plane_set):
    '''
    '''
    # 각 노드는 id와 point를 저장한다.
    node_save_info_list = []
    for node in node_set.nodes:
        save_info = {
            'idx': node.idx,
            'node_type':node.node_type,
            'point': node.point.tolist()
        }
        node_save_info_list.append(save_info)

    # 이를 저장한다.
    filename = os.path.join(output_path, 'node_set.json')
    with open(filename, 'w') as f:
        json.dump(node_save_info_list, f)


    # 각 라인은 id, points, to_node, from_node를 저장한다.
    line_save_info_list = []
    for line in line_set.lines:
        save_info = {
            'idx':line.idx,
            'from_node_idx':line.from_node.idx,
            'to_node_idx':line.to_node.idx,
            'points':line.points.tolist()
        }
        line_save_info_list.append(save_info)

    # 이를 저장한다.
    filename = os.path.join(output_path, 'line_set.json')
    with open(filename, 'w') as f:
        json.dump(line_save_info_list, f)


    # 각 Plane은 nodes를 저장한다
    plane_save_info_list = []
    for plane in plane_set.planes:
        
        node_index_list = []
        for node in plane.nodes:
            node_index_list.append(node.idx)

        save_info = {
            'idx':plane.idx,
            'node_idx_list':node_index_list
        }
        plane_save_info_list.append(save_info)


    # 이를 저장한다.
    filename = os.path.join(output_path, 'plane_set.json')
    with open(filename, 'w') as f:
        json.dump(plane_save_info_list, f)



def save_node_and_link(output_path, node_set, link_set):
    # save 시에는 항상 최신 포맷으로만 저장한다
    import subproc_save_link_ver2
    subproc_save_link_ver2.save_node_and_link(output_path, node_set, link_set)

