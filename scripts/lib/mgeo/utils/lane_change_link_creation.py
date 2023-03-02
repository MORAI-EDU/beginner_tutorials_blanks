#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs import *
import math

def calc_lane_change_path_list(src_link, max_lane_change):  
    '''
    A, B, C, D 와 같이 인접한 링크가 있을 때 (양방향으로 차선 변경 가능)
    차선 변경으로 이동 가능한 case를 생성한다.
    src_link는 시작하는 링크이고, max_lane_change는 최대 몇 번 차선 변경을 할지를 나타낸다
    
    예1) src_link = A, max_lane_change = 3라고 하면,
      결과는 다음과 같다 
      [(A, B), (A, B, C), (A, B, C, D)]

    예2) src_link = B, max_lane_change = 2라고 하면,
      [(B, A), (B, C), (B, C, D)]

    예3) src_link = C, max_lane_change = 1이라고 하면,
      [(C, B), (C, D)]
    ''' 
    lane_change_path_list = [] 


    # 차선 변경을 지원하지 않는 링크 타입이다.
    if src_link.link_type is not None:
        if src_link.link_type in ['1', '2', '3']:
            return []
            

    # STEP #1 왼쪽 방향 차선 변경으로 찾기
    # (바로 아래 오른쪽 방향 차선 변경으로 찾기와 동일)
    current_link = src_link
    for i in range(max_lane_change):
        # 현재 링크에서 차선 변경으로 진입 가능한 다음 링크가 있는지 확인
        print('type: {}',type(current_link))
        dst_link = current_link.get_left_lane_change_dst_link()
        if current_link.can_move_left_lane == False or dst_link is None:
            break
        
        # 있으면 차선 변경 링크의 경로에 현재 링크를 추가한다
        if i == 0:
            current_lane_ch = (src_link, dst_link)
        else:
            current_lane_ch += (dst_link, )
        lane_change_path_list.append(current_lane_ch)

        # 현재 링크 업데이트하기        
        current_link = dst_link 
    

    # STEP #2 오른쪽 방향 차선 변경으로 찾기
    # (바로 위 왼쪽 방향 차선 변경으로 찾기와 동일)
    current_link = src_link
    for i in range(max_lane_change):
        # 현재 링크에서 차선 변경으로 진입 가능한 다음 링크가 있는지 확인
        dst_link = current_link.get_right_lane_change_dst_link()
        if current_link.can_move_right_lane == False or dst_link is None:
            break
        
        # 있으면 차선 변경 링크의 경로에 현재 링크를 추가한다
        if i == 0:
            current_lane_ch = (src_link, dst_link)
        else:
            current_lane_ch += (dst_link, )
        lane_change_path_list.append(current_lane_ch)
        
        # 현재 링크 업데이트하기
        current_link = dst_link 

    return lane_change_path_list  


def create_lane_change_link(link_set, max_lane_change):
    '''
    최대 변경 가능한 차선의 수를 constant로 설정하여 차선 변경 링크를 생성한다.
    '''
    lane_ch_line_set = LineSet()
    
    # 모든 링크를 돌아본다
    for idx, src_link in link_set.lines.items():

        lane_ch_path_list = calc_lane_change_path_list(src_link, max_lane_change)
        for lane_ch_path in lane_ch_path_list:
            # src_link must be the same as lane_ch_path[0]
            dst_link = lane_ch_path[-1]
            
            idx = '{}-{}'.format(src_link.idx, dst_link.idx)
            lane_ch_link = Link(idx=idx, lazy_point_init=True)
            lane_ch_link.set_values_for_lane_change_link(lane_ch_path)
            lane_ch_line_set.append_line(lane_ch_link, create_new_key=False)

    return lane_ch_line_set


def create_lane_change_link_auto_depth_using_length(link_set, method, **kwargs):
    '''
    최대 변경 가능한 차선의 수를 constant로 설정하여 차선 변경 링크를 생성한다.
    usage:
      #1 : func_name(link_set, method=1, min_length_for_lane_change=20)
      #2 : func_name(link_set, method=2) >> 설정된 map value를 사용
    '''
    lane_ch_line_set = LineSet()
    
    # 모든 링크를 돌아본다
    for idx, src_link in link_set.lines.items():

        """ Method #1 : min_length_for_lane_change 를 이용하여 link_length / min_length_for_lane_change 로 계산하는 것"""
        if method == 1:
            min_length_for_lane_change = kwargs['min_length_for_lane_change'] # default value = 20
            if min_length_for_lane_change <= 0:        
                # 그냥 매우 큰 값을 입력하는 것
                max_lane_change = 50 
            else:
                link_length = src_link.get_total_distance()
                max_lane_change = int(math.floor(link_length / min_length_for_lane_change))
        else:
            raise BaseException('[ERROR] Invalid method number = {}'.format(method))


        lane_ch_path_list = calc_lane_change_path_list(src_link, max_lane_change)
        for lane_ch_path in lane_ch_path_list:
            # src_link must be the same as lane_ch_path[0]
            dst_link = lane_ch_path[-1]
            
            idx = '{}-{}'.format(src_link.idx, dst_link.idx)
            lane_ch_link = Link(idx=idx, lazy_point_init=True)
            lane_ch_link.set_values_for_lane_change_link(lane_ch_path)
            lane_ch_line_set.append_line(lane_ch_link, create_new_key=False)

    return lane_ch_line_set