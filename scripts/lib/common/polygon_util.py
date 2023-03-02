#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
mgeo_lib_path = os.path.normpath(os.path.join(current_path, '../mgeo/'))
# print('mgeo_lib_path: {}'.format(mgeo_lib_path))
sys.path.append(mgeo_lib_path)

import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt

from class_defs import *
import math

def minimum_bounding_rectangle(points):
    # https://stackoverflow.com/questions/13542855/algorithm-to-find-the-minimum-area-rectangle-for-given-points-in-order-to-comput/33619018#33619018
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.

    :param points: an nx2 matrix of coordinates
    :rval: an nx2 matrix of coordinates
    """
    from scipy.ndimage.interpolation import rotate
    pi2 = np.pi/2.
    
    # 2차원배열만됨
    # x,y 중복되면 안됨
    # get the convex hull for the points

    hull_points = points
    # hull_points = points[ConvexHull(points).vertices]

    cut_points = []
    
    for point in hull_points:
        if len(point) > 2:
            cut_points.append(np.array([point[0],point[1]]))
        else:       
            cut_points.append(point)
    
    cut_points = np.array(cut_points)

    # calculate edge angles
    edges = np.zeros((len(cut_points)-1, 2))
    edges = cut_points[1:] - cut_points[:-1]

    angles = np.zeros((len(edges)))
    angles = np.arctan2(edges[:, 1], edges[:, 0])

    angles = np.abs(np.mod(angles, pi2))
    angles = np.unique(angles)

    # find rotation matrices
    # XXX both work
    rotations = np.vstack([
        np.cos(angles),
        np.cos(angles-pi2),
        np.cos(angles+pi2),
        np.cos(angles)]).T
    rotations = rotations.reshape((-1, 2, 2))

    # apply rotations to the hull
    rot_points = np.dot(rotations, cut_points.T)

    # find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # find the box with the best area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # return the best box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    r = rotations[best_idx]

    rval = np.zeros((4, 2))
    rval[0] = np.dot([x1, y2], r)
    rval[1] = np.dot([x2, y2], r)
    rval[2] = np.dot([x2, y1], r)
    rval[3] = np.dot([x1, y1], r)

    returnValue = np.zeros((4,3))
    i = 0
    for value in rval:
        result = []
        for point in hull_points:
            result.append((((value[0] - point[0] )**2) + ((value[1]-point[1])**2) )**0.5)
        minindex = np.argmin(np.array(result))
        
        returnValue[i]= np.array([value[0], value[1], hull_points[minindex][2]])
       
        i = i+1
        
    return returnValue

def calculate_centroid(points):
    sx = sy= sz = sL = 0
    for i in range(len(points)):
        x0, y0, z0 = points[i - 1]     # in Python points[-1] is last element of points
        x1, y1, z1 = points[i]
        L = ((x1 - x0)**2 + (y1 - y0)**2 + (z1-z0)**2) ** 0.5
        sx += (x0 + x1)/2 * L
        sy += (y0 + y1)/2 * L
        sz += (z0 + z1)/2 * L
        sL += L
        
    centroid_x = sx / sL
    centroid_y = sy / sL
    centroid_z = sz / sL

    print('cent x = %f, cent y = %f, cent z = %f'%(centroid_x, centroid_y, centroid_z))

    # TODO: 계산하는 공식 추가하기
    return np.array([centroid_x,centroid_y, centroid_z])

def sorted_points(points):
    xs = []
    xy = []
    for i in points:
        xs.append(i[0])
        xy.append(i[1])

    harf_x = (max(xs) + min(xs)) / 2 
    
    # 겹치는 x값이 있는지 확인
    if harf_x in xs:
        harf_x = harf_x + 0.001
    
    y_right = []
    y_left = []

    #harf_x보다 x값이 작은 좌표집합과 큰 좌표집합으로 분리
    for i in points:
        if i[0] < harf_x:
            y_left.append(i)
        else: 
            y_right.append(i)

    y_left.sort(key=lambda x : x[1] , reverse = False)
    y_right.sort(key=lambda x : x[1] , reverse = True)

    return np.array(y_left + y_right)


def calculate_heading(traffic_set):
    related_link = Line()
    for idx, item in traffic_set.signals.items():
    # 표지판/신호등이 참조하는 link 가져오기 

        if len(item.link_list) == 0:
            raise BaseException('ERROR: No link_list')
        related_link = item.link_list[0]
    
    # link의 평균 벡터(시작점 -> 끝점)을 이용, heading을 계산
    link_avg_vector = related_link.points[-1] - related_link.points[0]
    signal_dir_vector = -1 * link_avg_vector # 표지판/신호등은 도로와 반대 방향
    signal_heading_deg = np.arctan2(signal_dir_vector[1], signal_dir_vector[0]) * 180 / np.pi
    # signal_heading_rad = math.radians(signal_heading_deg)
    
    simulatorHeadingValue = signal_heading_deg + 180
    if simulatorHeadingValue > 360 :
        simulatorHeadingValue = simulatorHeadingValue - 360
    else:
        simulatorHeadingValue = signal_heading_deg

    # orientation 입력
    orientation_string = '0.0/{:.6f}/0.0'.format(simulatorHeadingValue)

    return orientation_string

def __create_traffic_sign_set_from_shp(sf, origin, link_set):   
    traffic_sign_set = SignalSet()

    shapes = sf.shapes()
    records  = sf.records()

    if len(shapes) != len(records):
        raise BaseException('[ERROR] len(shapes) != len(records)')
    
    for i in range(len(shapes)):
        shp_rec = shapes[i]
        dbf_rec = records[i]

        # Convert to numpy array
        shp_rec.points = np.array(shp_rec.points)
        shp_rec.z = np.array(shp_rec.z)

        # Point에 z축 값도 그냥 붙여버리자
        shp_rec.points = np.c_[shp_rec.points, shp_rec.z]

        # origin 무조건 전달, 상대좌표로 변경
        shp_rec.points -= origin

        # id 관련 변수는 전부 string으로 처리
        signal_id = to_str_if_int(dbf_rec['ID'])
        link_id_list = [to_str_if_int(dbf_rec['LinkID'])]

        signal = Signal(signal_id)
        signal.link_id_list = link_id_list

        # if dbf_rec['LinkID']:
        #     signal.link_id_list = dbf_rec['LinkID'].split(',') 

        signal.dynamic = False
        signal.orientation = '+'
        signal.country = 'KR'

        # LINK ID List를 사용해서 Link 참조 List 구성     
        for link_id in signal.link_id_list:
            if link_id in link_set.lines.keys():
                link = link_set.lines[link_id]
                signal.add_link_ref(link)
            else:
                print('[ERROR] Cannot find Link (id={}) for TS (id={}) Skipping this one'.format(link_id, signal_id))
                # raise BaseException('[ERROR] Could not find link ID mapped in link set.')
    
        for link in signal.link_list:
            if signal.road_id == None or signal.road_id == '' :
                signal.road_id = link.road_id
            else:
                # Signal이 참조하고 있는 lane들이 서로 다른 road id를 가진 경우 예외 발생
                if signal.road_id != link.road_id:
                    raise BaseException('[ERROR] The lanes referenced by signal have different road id.')

        signal.type = dbf_rec['Type']
        signal.sub_type = dbf_rec['SubType']

        # 사이즈 설정
        # type, sub_type 값을 설정한 후 호출해야 함
        signal.set_size()

        # 최고속도제한 규제표지
        if signal.type == '2' and signal.sub_type == '224' and len(signal.link_list) > 0  :
            for link in signal.link_list:
                if link.idx == signal.link_id_list[0]:
                    if link.max_speed_kph == 0:
                        signal.value = 50
                    else:
                        signal.value = link.max_speed_kph
                    
            # signal.value = signal.link_list[signal.link_id_list[0]].max_speed_kph
        # 최저속도제한 규제표지
        elif signal.type == '2' and signal.sub_type == '225' and len(signal.link_list) > 0 :
            for link in signal.link_list:
                if link.idx == signal.link_id_list[0]:
                    if link.min_speed_kph == 0:
                        signal.value = 30
                    else:
                        signal.value = link.min_speed_kph
                    

            # signal.value = signal.link_list[signal.link_id_list[0]].min_speed_kph
            
        signal.point = shp_rec.points[0]
        
        traffic_sign_set.signals[signal.idx] = signal
    
    return traffic_sign_set

def __create_traffic_light_set_from_shp(sf, origin, link_set):
    traffic_light_set = SignalSet()

    shapes = sf.shapes()
    records  = sf.records()

    if len(shapes) != len(records) :
        raise BaseException('[ERROR] len(shapes) != len(records)')
    
    for i in range(len(shapes)):
        shp_rec = shapes[i]
        dbf_rec = records[i]

        # Convert to numpy array
        shp_rec.points = np.array(shp_rec.points)
        shp_rec.z = np.array(shp_rec.z)

        # Point에 z축 값도 그냥 붙여버리자
        shp_rec.points = np.c_[shp_rec.points, shp_rec.z]

        # origin 무조건 전달, 상대좌표로 변경
        shp_rec.points -= origin

        # id 관련 변수는 전부 string으로 처리
        signal_id = to_str_if_int(dbf_rec['ID'])
        link_id_list = [to_str_if_int(dbf_rec['LinkID'])]

        signal = Signal(signal_id)
        signal.link_id_list = link_id_list

        # if dbf_rec['LinkID'] :
        #     signal.link_id_list = dbf_rec['LinkID'].split(',') 

        signal.dynamic = True
        signal.orientation = '+'
        signal.country = 'KR'

        # LINK ID List를 사용해서 Link 참조 List 구성
        for link_id in signal.link_id_list:
            if link_id in link_set.lines.keys():
                link = link_set.lines[link_id]
                signal.add_link_ref(link)
            else:
                print('[ERROR] Cannot find Link (id={}) for TL (id={}) Skipping this one'.format(link_id, signal_id))

        for link in signal.link_list:
            if signal.road_id == None or signal.road_id == '' :
                signal.road_id = link.road_id
            else:
                # Signal이 참조하고 있는 lane들이 서로 다른 road id를 가진 경우 예외 발생
                if signal.road_id != link.road_id :
                    raise BaseException('[ERROR] The lanes referenced by signal have different road id.')             

        signal.type = dbf_rec['Type']
        signal.sub_type = ''

        # 사이즈 설정
        # type, sub_type 값을 설정한 후 호출해야 함
        signal.set_size()

        signal.point = shp_rec.points[0]
        
        traffic_light_set.signals[signal.idx] = signal
    
    return traffic_light_set        



    
def to_str_if_int(val):
    # list인 경우 먼저 체크
    if isinstance(val, list):
        ret_list = list()
        for each_val in val:
            if isinstance(each_val, int):
                ret_list.append(str(each_val))
            else:
                ret_list.append(each_val)
        return ret_list

    # 단일 값인 경우
    if isinstance(val, int):
        return str(val)
    else:
        return val
