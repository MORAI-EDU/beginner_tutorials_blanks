#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from utils.logger import Logger

import matplotlib.pyplot as plt
import numpy as np 
from class_defs.base_plane import BasePlane
import lib.common.polygon_util as util

class SurfaceMarking(BasePlane):
    """
    노면표시를 나타내는 클래스. 두 가지 역할을 수행한다
    1) Mesh 생성 (예: Speedbump Mesh Guide 생성)
    2) PlannerMap에서 해당 표시를 인식 (현재 링크와 관련 있는 노면 표시를 조회 가능)
    """
    def __init__(self, points=None, idx=None):
        super(SurfaceMarking, self).__init__(points, idx)
        
        self.link_id_list = []
        self.road_id = ''

        # 참조를 저장하는 변수
        # NOTE: 참조를 위한 key가 있어도 참조할 인스턴스가 존재할 수 있다
        #       >> 이 경우 len(self.link_id_list) != len(self.link_list)이다
        self.link_list = list()

        # 기타 속성 정보
        self.type = None
        self.sub_type = None
        self.type_code_def = '' 

        """이하는 MPL에서의 draw를 위함"""
        # matplotlib에 의해 그려진 list of Line2D 객체에 대한 레퍼런스
        # plt.plot을 호출하면서 반환되는 값이며,
        # 이 레퍼런스를 통해 matplotlib에서 삭제 또는 스타일 변경 등이 가능
        self.plotted_obj = None

        # Visualization 모드
        self.reset_vis_mode_manual_appearance()


    def add_link_ref(self, link):
        if link not in self.link_list:
            self.link_list.append(link)
        
        if self not in link.surface_markings:
            link.surface_markings.append(self)


    def create_mesh_gen_points(self):
        mesh_gen_vertices = []
        mesh_gen_vertex_subsets_for_each_face = []

        # 우선 처음 point와 마지막 point가 같은 포인트인지 확인한다
        diff = self.points[0] - self.points[-1]
        dist = np.linalg.norm(diff, ord=2)
        if dist > 0.01: # 두 point 사이 거리가 1cm 이상이면 오류
            raise BaseException('Error in the sm: {}, the first point and the last point are not the same. dist: {}'.format(self.idx, dist))

        # 마지막 point를 제외하고 전달. 이 때 numpy.array를 기본 python list로 변경한다
        # vtk에서는 numpy array를 지원하지 않기 때문.
        
        # 다각형일 경우 사각형으로 만드는 로직 추가(MS-60)
        points = util.minimum_bounding_rectangle(self.points)

        # mesh_gen_vertices = self.points.tolist()[:-1]
        mesh_gen_vertices = points.tolist()

        

        # 기본 surface_marking에서는 인스턴스 하나 당 face가 하나 생성되면 된다.
        # NOTE: 마지막 포인트는 채울 필요가 없다
        vertex_index_for_face = []
        for i in range(len(mesh_gen_vertices)):
            vertex_index_for_face.append(i)

        # 만일 인스턴스 하나 당 face가 여러개면, 아래와 같은 부분이 여러번 호출된다
        mesh_gen_vertex_subsets_for_each_face.append(vertex_index_for_face)

        return mesh_gen_vertices, mesh_gen_vertex_subsets_for_each_face


    def draw_plot(self, axes):
        # 그려야하는 width와 color가 지정되어 있으면 해당 값으로만 그린다
        if self.vis_mode_line_width is not None and \
            self.vis_mode_line_color is not None:
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                linewidth=self.vis_mode_line_width,
                color=self.vis_mode_line_color,
                markersize=1,
                marker='o')
            return
        
        else:
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                markersize=1,
                marker='o',
                color='b')


    def erase_plot(self):
        if self.plotted_obj is not None:
            # list of matplotlib.lines.Line2D 이므로
            # iterate 하면서 remove를 호출해야 함
            for obj in self.plotted_obj:
                if obj.axes is not None:
                    obj.remove()


    def hide_plot(self):
        if self.plotted_obj is not None:
            for obj in self.plotted_obj:
                obj.set_visible(False)


    def unhide_plot(self):
        if self.plotted_obj is not None:
            for obj in self.plotted_obj:
                obj.set_visible(True)            


    def set_vis_mode_manual_appearance(self, width, color):
        self.vis_mode_line_width = width
        self.vis_mode_line_color = color


    def reset_vis_mode_manual_appearance(self):
        self.set_vis_mode_manual_appearance(None, None)      


    def fill_in_points_evenly(self, step_len):
        new_points = self.calculate_evenly_spaced_link_points(step_len)
        self.set_points(new_points)


    def calculate_evenly_spaced_link_points(self, step_len):
        '''
        현재의 링크를 일정한 간격으로 채워주는 점의 집합을 계산한다
        '''
        new_points_all = self.points[0]

        skip_getting_new_point = False
        for i in range(len(self.points) - 1):
            # 시작점을 어디로 할 것인가를 결정
            # 만일 지난번 point에서 예를 들어
            # x = 0, 3, 6, 9 까지 만들고, 
            # 원래는 x = 10까지 와야하는 상황이었다.
            # 실제로 포인트는 x = 9에만 찍혀있다.
            # 따라서 여기부터 시작하는 것이 옳다.
            if not skip_getting_new_point:
                point_now  = self.points[i]


            point_next = self.points[i + 1]

            # 2. point_now에서 point_next까지 point를 생성한다
            # 2.1) 1 step 에 해당하는 벡터 
            dir_vector = point_next - point_now
            mag = np.linalg.norm(dir_vector, ord=2)
            unit_vect = dir_vector / mag
            step_vect = step_len * unit_vect
            
            # 2.2) 벡터를 몇 번 전진할 것인가
            cnt = (int)(np.floor(mag / step_len))
            if cnt == 0:
                # 마지막보다 이전이면, 즉 현재포인트와 다음 포인트가 너무 가깝다
                if i < len(self.points) - 2:
                    #만일 이렇게 되면, 다음 포인트를 그냥 여기 포인트로 하는게 낫다
                    # 현재 point_now를 그 다음 point_now로 그대로 사용한다
                    skip_getting_new_point = True
                    continue
                else:
                    # 마지막인데, 진짜 마지막 포인트가 너무 짧은 거리에 있다
                    # 그냥 붙여주고 끝낸다
                    new_points_all = np.vstack((new_points_all, point_next))
                    break
                

            # 2.3) 현재 위치는 포함하지 않고, 새로운 포인트를 cnt 수 만큼 생성, 전체 포인트에 연결
            new_points = self._create_points_using_step(point_now, step_vect, cnt)
            new_points_all = np.vstack((new_points_all, new_points))

            # 2.4) 원래 있는 point 사이의 길이가 mag로 나눠서 딱 떨어지면
            #      마지막 포인트가 포함되었다. 이에 따라 처리가 달라짐
            if mag % step_len == 0:
                # 이렇게되면, 끝 점이 자동으로 포함될 것이다 
                pass
            else:
                #만일 이렇게 되면, 다음 포인트를 그냥 여기 포인트로 하는게 낫다
                skip_getting_new_point = True
                point_now = new_points_all[-1]

                # 2.5) 마지막인 경우, 진짜 마지막 포인트를 강제로 넣는다
                if i == len(self.points) - 2:
                    new_points_all = np.vstack((new_points_all, point_next))

        return new_points_all


    def _create_points_using_step(self, current_pos, xyz_step_size, step_num):
        next_pos = current_pos

        if step_num == 0:
            # raise Exception('[ERROR] Cannot connect the two points, check your unit\
            #     vector step length')
            ret = np.array(next_pos)

        else:
            for i in range(step_num):
                
                next_pos = [next_pos[0] + xyz_step_size[0],
                    next_pos[1] + xyz_step_size[1],
                    next_pos[2] + xyz_step_size[2]]
                if i == 0:
                    ret = np.array(next_pos)
                else:
                    ret = np.vstack((ret, next_pos))

        return ret


    @staticmethod
    def to_dict(obj):
        """json 파일 등으로 저장할 수 있는 dict 데이터로 변경한다"""

        dict_data = {
            'idx': obj.idx,
            'points': obj.points.tolist(),
            'link_id_list': obj.link_id_list,
            'road_id' : obj.road_id,
            'type': obj.type,
            'sub_type': obj.sub_type
        }
        return dict_data


    @staticmethod
    def from_dict(dict_data, link_set=None):
        """json 파일등으로부터 읽은 dict 데이터에서 Signal 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""
        # 필수 정보
        idx = dict_data['idx']
        points = np.array(dict_data['points'])

        # 연결된 객체 참조용 정보
        link_id_list = dict_data['link_id_list']
        road_id = dict_data['road_id']

        # 기타 속성 정보
        sm_type = dict_data['type'] # type은 지정된 함수명이므로 혼란을 피하기 위해 sign_type으로
        sm_subtype = dict_data['sub_type']

        """STEP #2 인스턴스 생성"""
        obj = SurfaceMarking(points=points, idx=idx)

        # 연결된 객체 참조용 정보
        obj.link_id_list = link_id_list
        obj.road_id = road_id

        # 기타 속성 정보
        obj.type = sm_type
        obj.sub_type = sm_subtype

        """STEP #3 인스턴스 참조 연결"""
        if link_set is not None:
            for link_id in link_id_list:
                if link_id in link_set.lines.keys():
                    link = link_set.lines[link_id]
                    obj.add_link_ref(link)

        return obj
