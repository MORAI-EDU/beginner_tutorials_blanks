#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs import *

import numpy as np
import copy

from logger import Logger


def find_dangling_nodes(node_set):
    """어디에도 연결되어있지 않은 노드를 찾는다"""
    if node_set is None:
        raise BaseException('None is passed for an argument link_set')

    dangling_nodes = list()

    for idx, node in node_set.nodes.items():
        if node.is_dangling_node():
            dangling_nodes.append(node)

    return dangling_nodes


def find_dangling_links(link_set):
    """FROM_Node 또는 TO_NODE 않은 링크를 찾는다"""
    if link_set is None:
        raise BaseException('None is passed for an argument link_set')

    dangling_links = list()

    for idx, link in link_set.lines.items():
        if link.is_dangling_link():
            dangling_links.append(link)

    return dangling_links


def fix_dangling_links(dangling_link_set, node_set): 
    for link in dangling_link_set:
        start_node = Node()
        start_node.point = link.points[0]
        node_set.append_node(start_node, True)

        end_node = Node()
        end_node.point = link.points[len(link.points) - 1]
        node_set.append_node(end_node, True)

        link.set_from_node(start_node)
        link.set_to_node(end_node)

        Logger.log_info('Dangling links fixed: {}'.format(link.idx))


def check_for_node_connected_link_not_included_in_the_link_set(node_set, link_set):
    for var in node_set.nodes:
        if isinstance(node_set.nodes, dict):
            node = node_set.nodes[var]
        else:
            node = var

        for link in node.get_to_links() + node.get_from_links():
            if link.idx not in link_set.lines:
                raise BaseException('link.idx = {} not in self.link_set.links'.format(link.idx))


def search_overlapped_node(node_set, dist_threshold):
    """
    같은 위치에 있는 노드를 검색해준다
    """
    # 참고: deepcopy해서 해결할 방법이 없을까 싶었는데,
    # 그냥 skip_flag를 사용하기로 함
    # node_set = copy.deepcopy(_node_set)

    
    # 우선 skip_flag를 만든다
    skip_flags = {}
    for idx, node in node_set.nodes.items():
        skip_flags[idx] = 0
    
    # 최종 반환할 값: 수정해야 하는 모든 set이 여기에 저장된다
    overlapped_node_set = []

    # 이제 하나씩 검색을 해본다
    for idx, node in node_set.nodes.items():          
        # 나 자신은 검색하지 않아도 되므로 다음과 같이 변경
        skip_flags[idx] = 1

        # 다른 노드를 검색해본다
        # 현재 노드와 위치가 같은 모든 노드를 검색한다
        one_set = [node, ]
        for another_idx, another_node in node_set.nodes.items():
            # skip_flag 또한 0일 경우에만 확인하면 된다. (현재 노드의 경우 1로 체크가 되었으므로 skip된다)
            if skip_flags[another_idx] == 0:
                pos_vector = another_node.point - node.point
                dist = np.linalg.norm(pos_vector, ord=2)
                
                if dist < dist_threshold:
                    one_set.append(another_node)
                    skip_flags[another_idx] = 1

        # 겹치는 노드가 발견이 된 경우
        if len(one_set) > 1:
            str_id_list = '['
            for n in one_set:
                str_id_list += '{}, '.format(n.idx)
            str_id_list += ']'
            Logger.log_info('Overlapped node found: {}'.format(str_id_list))
            overlapped_node_set.append(one_set)
    
    # print('----------------------------------')
    return overlapped_node_set


def repair_overlapped_node(overlapped_node_set):
    '''
    겹치는 노드가 있으면, 한 노드에만 연결되도록 수정해준다
    쓸모가 없어진 노드들의 리스트가 반환된다
    '''
    nodes_of_no_use = []
    for nodes in overlapped_node_set:
        # 가장 처음에 있는 값을 기준으로 맞춘다
        node_fix = nodes[0]

        for i in range(1, len(nodes)):
            from_links = nodes[i].get_from_links() 
            to_links = nodes[i].get_to_links()

            # for link in from_links:
            for link in copy.copy(from_links):
                # 이 노드로 들어가던 링크들은, to_link를 node_fix로 바꾸어야 한다
                link.set_to_node(node_fix)

            # for link in to_links:
            for link in copy.copy(to_links):
                # 이 노드에서 나오던 링크들은, from_link를 node_fix로 바꾸어야 한다
                link.set_from_node(node_fix)

            # nodes[i]의 to_links, from_links를 초기화한다
            nodes[i].to_links = list()
            nodes[i].from_links = list() 
            nodes_of_no_use.append(nodes[i])

        # [INFO] only for print
        nodes_str = '['
        for node in nodes:
            nodes_str += '{}, '.format(node.idx)
        nodes_str += ']'
        Logger.log_info('Overlapped nodes fixed: {} => {}'.format(nodes_str, node_fix.idx))

    # print('----------------------------------')
    return nodes_of_no_use
            

def delete_nodes_from_node_set(node_set, nodes_to_delete):
    if isinstance(node_set.nodes, dict):
        for node in nodes_to_delete:
            node_set.nodes.pop(node.idx)

    else:
        # TODO: implement this function for node_set.nodes is a list
        raise NotImplementedError('[ERROR]')


def _find_nearest_node_in_range(point, node_set, distance_using_xy_only, dist_threshold=0.1):
    # 가장 가까운 노드와, 노드까지의 거리를 찾는다
    min_dist = np.inf
    nearest_node = None


    for var in node_set.nodes:
        if isinstance(node_set.nodes, dict):
            # 이 경우 var이 key 이다.
            node = node_set.nodes[var]
        elif isinstance(node_set.nodes, list):
            # 이 경우 var이 node이다.
            node = var
        else:
            raise BaseException('[ERROR] Unexpected type for node_set.nodes')
        
        if distance_using_xy_only:
            pos_vect = node.point[0:2] - point[0:2]
        else:
            pos_vect = node.point - point
        dist = np.linalg.norm(pos_vect, ord=2)
        if dist < min_dist:
            min_dist = dist
            nearest_node = node
    
    # 가장 가까운 노드까지의 거리가 정한 값보다 작으면, 적합한 노드가 있는 것이다
    if min_dist < dist_threshold:
        return True, nearest_node
    else:
        return False, None


def search_for_a_to_node_and_set(link, node_set, to_node):
    # to node를 찾아준다
    point = link.points[-1] # 링크의 끝점
    find_result, to_node = _find_nearest_node_in_range(point, node_set, distance_using_xy_only=False)

    # 찾은게 없으면 노드 새로 만들기
    if not find_result:
        # 이 때 idx는 현재 있는 노드 수를 이용해서 만든다
        new_node_idx = len(node_set.nodes)
        # Logger.log_info('Adding a new node. new id: {}'.format(new_node_idx))

        to_node = Node(new_node_idx)
        to_node.point = point
        node_set.nodes[to_node.idx] = to_node


    # 찾은 노드 또는 새로 만든 노드를 추가
    link.set_to_node(to_node)

    return find_result, to_node.idx


def search_for_a_from_node_and_set(link, node_set, from_node):
    # from node를 찾아준다
    point = link.points[0] # 링크의 시작점
    find_result, from_node = _find_nearest_node_in_range(point, node_set, distance_using_xy_only=True)

    # 찾은게 없으면 노드 새로 만들기
    if not find_result:
        # 이 때 idx는 현재 있는 노드 수를 이용해서 만든다
        new_node_idx = len(node_set.nodes)
        # Logger.log_info('Adding a new node. new id: {}'.format(new_node_idx))

        from_node = Node(new_node_idx)
        from_node.point = point
        node_set.nodes[from_node.idx] = from_node


    # 찾은 노드 또는 새로 만든 노드를 추가
    link.set_from_node(from_node)

    return find_result, from_node.idx


def change_all_item_id_to_string(mgeo_planner_map):
    """
    내부의 id 데이터 중 string이 아닌 것이 있으면 모두 string 또는 string의 list로 수정한다.
    체크하는 데이터의 종류는 아래와 같다.
    1) mgeo 데이터 자체의 id
    2) 참조에 사용하는 데이터의 id 값 (id_list도 포함)
    """
    node_set = mgeo_planner_map.node_set
    link_set = mgeo_planner_map.link_set
    ts_set = mgeo_planner_map.sign_set
    tl_set = mgeo_planner_map.light_set
    sm_set = mgeo_planner_map.sm_set
    jc_set = mgeo_planner_map.junction_set


    for idx, obj in node_set.nodes.items():
        if not isinstance(obj.idx, str):
            Logger.log_info('node id: {} >> id fixed.'.format(idx))
            obj.idx = str(obj.idx)
            

    for idx, obj in link_set.lines.items():
        if not isinstance(obj.idx, str):
            Logger.log_info('link id: {} >> id fixed.'.format(idx))
            obj.idx = str(obj.idx)
            
        if not isinstance(obj.road_id, str):
            Logger.log_info('link id: {} >> road_id: {} fixed.'.format(idx, obj.road_id))
            obj.road_id = str(obj.road_id)
            

    for idx, obj in ts_set.signals.items():
        if not isinstance(obj.idx, str):
            Logger.log_info('traffic sign id: {} >> id fixed.'.format(idx))
            obj.idx = str(obj.idx)

        if not isinstance(obj.road_id, str):
            Logger.log_info('traffic sign id: {} >> road_id: {} fixed.'.format(idx, obj.road_id))
            obj.road_id = str(obj.road_id)

        for i in range(len(obj.link_id_list)):
            link_id = obj.link_id_list[i]
            if not isinstance(link_id, str):
                Logger.log_info('traffic sign id: {} >> link_id_list[{}] = {} fixed.'.format(idx, i, link_id))
                obj.link_id_list[i] = str(link_id)


    for idx, obj in tl_set.signals.items():
        if not isinstance(obj.idx, str):
            Logger.log_info('traffic light id: {} >> id fixed.'.format(idx))
            obj.idx = str(obj.idx)

        if not isinstance(obj.road_id, str):
            Logger.log_info('traffic light id: {} >> road_id: {} fixed.'.format(idx, obj.road_id))
            obj.road_id = str(obj.road_id)

        for i in range(len(obj.link_id_list)):
            link_id = obj.link_id_list[i]
            if not isinstance(link_id, str):
                Logger.log_info('traffic sign id: {} >> link_id_list[{}] = {} fixed.'.format(idx, i, link_id))
                obj.link_id_list[i] = str(link_id)


    for idx, obj in sm_set.data.items():
        if not isinstance(obj.idx, str):
            Logger.log_info('surface marking id: {} >> id fixed.'.format(idx))
            obj.idx = str(obj.idx)

        if not isinstance(obj.road_id, str):
            Logger.log_info('surface marking id: {} >> road_id: {} fixed.'.format(idx, obj.road_id))
            obj.road_id = str(obj.road_id)

        for i in range(len(obj.link_id_list)):
            link_id = obj.link_id_list[i]
            if not isinstance(link_id, str):
                Logger.log_info('traffic sign id: {} >> link_id_list[{}] = {} fixed.'.format(idx, i, link_id))
                obj.link_id_list[i] = str(link_id)


    for obj in jc_set.junctions.values():
        if not isinstance(obj.idx, str):
            Logger.log_info('junction id: {} >> id fixed.'.format(idx))
            obj.idx = str(obj.idx)