#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from utils.logger import Logger

import matplotlib.pyplot as plt
import numpy as np 
from .line import Line

from collections import OrderedDict

class LaneMarking(Line):
    """도로의 차선을 표현하는 선, Mesh 생성을 위해 사용된다."""
    def __init__(self, points=None, idx=None):
        super(LaneMarking, self).__init__(points, idx)

        # Visualization 모드
        self.set_vis_mode_all_different_color(True)

        self.lane_code_def = ''
        self.lane_code = 0

        self.lane_color = ''
        self.lane_shape = []

        # dash solid_line_interval
        self.lane_width = 0.15 # U턴구역선이면 0.35로 변경할 것
        self.dash_interval_L1 = 30 # 도색 길이
        self.dash_interval_L2 = 50 # 빈 길이
        self.double_line_interval = 0.10 # 겹선일 때 두 선 사이의 거리, 규성상 0.10~0.15, 차선 종류와 관계없이 일정.

        # mesh를 생성하기 위한 vertices
        self.mesh_gen_vertices = [] 
        self.mesh_gen_vertex_subsets_for_each_face = [] # face를 구성할 vertices의 index를 기록한다

        # 
        self.vis_mode_marker_size = 0
        self.vis_mode_marker_style = ""


    def get_lane_num(self):
        return len(self.lane_shape)


    def is_every_attribute_equal(self, another):
        """attribute가 같은지 확인한다"""
        if self.lane_code_def != another.lane_code_def:
            return False
        
        if self.lane_code != another.lane_code:
            return False

        if self.lane_color != another.lane_color:
            return False

        if self.lane_shape != another.lane_shape:
            return False

        if self.lane_width != another.lane_width:
            return False

        if self.dash_interval_L1 != another.dash_interval_L1:
            return False

        if self.dash_interval_L2 != another.dash_interval_L2:
            return False

        if self.double_line_interval != another.double_line_interval:
            return False
        
        return True

  
    def get_attribute_from(self, src):
        LaneMarking.copy_attribute(src, self)


    def to_dict(self):
        dict_data = {
            'idx': self.idx,
            'from_node_idx': self.from_node.idx if self.from_node else None,
            'to_node_idx': self.to_node.idx if self.to_node else None,
            'points': self.points.tolist(),
            'lane_code_def': self.lane_code_def,
            'lane_code': self.lane_code,
            'lane_color': self.lane_color,
            'lane_shape': self.lane_shape,
            'lane_width': self.lane_width,
            'dash_interval_L1': self.dash_interval_L1,
            'dash_interval_L2': self.dash_interval_L2,
            'double_line_interval': self.double_line_interval
        }
        return dict_data


    def create_mesh_gen_points(self, solid_line_interval=0.5):

        if self.get_lane_num() == 1:
            return self.create_mesh_gen_points_single_lane(solid_line_interval)
        else:
            return self.create_mesh_gen_points_double_lane(solid_line_interval)


    def create_mesh_gen_points_single_lane(self, solid_line_interval=0.5):
        """일정 간격으로 mesh gen point를 생성한다. 이 때, 본 클래스의 point는 0.1m 간격으로 채워져있어야한다.
        mesh gen point란, points를 기준으로, mesh를 생성할 수 있는 point를 차선을 기준으로 위 아래에 생성하는 것이다.
      

        [single lane이면]

        (0) <-- 0.5m --> (2) <-- 0.5m --> (4)
        --------------------------------------
        (1)              (3)              (5) 
        """
        self.mesh_gen_vertices = []
        self.mesh_gen_vertex_subsets_for_each_face = []

        # 
        point_step = 0.1

        # 실선일 때
        solid_only_step = int(solid_line_interval/point_step)

        below_exec = False

        if 'Broken' in self.lane_shape:
            # 점선일 때는 다음의 계산이 필요 
            dash_interval_L1 = self.dash_interval_L1
            dash_interval_L2 = self.dash_interval_L2
            L1L2 = dash_interval_L1 + dash_interval_L2
            if L1L2 == 0.0:
                Logger.log_error('lane marking init has problem')
                return self.mesh_gen_vertices, self.mesh_gen_vertex_subsets_for_each_face

            # 남는 길이를 계산하고, 이를 다시 적당히 분배해야 한다
            total_length = len(self.points) * point_step
            total_num_set = int(np.floor(total_length / L1L2))
            if total_num_set == 0: # delta 계산을 위해 total_num_set으로 나눠야하므로 계산 필요
                Logger.log_warning('lane: {} is too short to create a mesh (lane code: {}, total len: {:.2f}, L1: {}, L2: {})'.format(self.idx, self.lane_code, total_length, dash_interval_L1, dash_interval_L2))
                return self.mesh_gen_vertices, self.mesh_gen_vertex_subsets_for_each_face
            remainder = total_length % L1L2
            delta = remainder / (total_num_set * 2)
            
            
            # 이만큼을 각 점선 길이에 더해주면 된다
            dash_interval_L1 += delta
            dash_interval_L2 += delta

            # 이를 point index 단위로 바꾸면 아래와 같다 (ceil, floor를 일부러 교차함)
            dashed_step_L1 = int(np.ceil(dash_interval_L1/point_step))
            dashed_step_L2 = int(np.floor(dash_interval_L2/point_step))

            below_exec = True


        i = 0
        face_cnt = 0
        paint = True # 점선일 때, 그리는 구간 / 넘어가는 구간을 토글링하기 위한 변수
        
        while i < len(self.points):

            """STEP #01 vertex 생성을 위해 현재 lane marking에서 수직한 벡터 계산"""
            # 현재 위치에서의 벡터 계산
            if i == len(self.points) - 1: # 마지막 값이면, 이전 값과의 차이로 계산
                vect = self.points[i] - self.points[i-1]
            else:
                vect = self.points[i+1] - self.points[i]
            vect = vect / np.linalg.norm(vect, ord=2)

            # 현재 벡터를 xy평면상에서 +-90deg 회전시킨다.
            pos_vect_ccw = self.rorate_around_z_axis(np.pi/2, vect)
            pos_vect_cw = self.rorate_around_z_axis(-np.pi/2, vect)
            
            # 여기서 z값은 그냥 0으로 한다
            pos_vect_ccw[2] = 0
            pos_vect_cw[2] = 0


            """STEP #02 Mesh 생성을 위한 vertex 생성하기"""
            # 위 아래로 width/2 위/아래로 움직이면 된다.
            pos_vect_ccw = pos_vect_ccw * self.lane_width / 2.0
            pos_vect_cw = pos_vect_cw * self.lane_width / 2.0

            # 이를 현재 위치에서 더해주면 된다. 이 때 numpy array가 아닌 일반 list로 저장한다 (vtk에서 numpy array 지원X)
            up = (self.points[i] + pos_vect_ccw).tolist()
            dn = (self.points[i] + pos_vect_cw).tolist()

            self.mesh_gen_vertices.append(up)
            self.mesh_gen_vertices.append(dn)

            if self.lane_shape[0] == 'Solid':
                """ Case #A 단선, 실선인 경우"""
                i += solid_only_step
            else:
                if not below_exec:
                    Logger.log_debug('WTF')
                """ Case #B 단선, 점선인 경우"""
                if paint:
                    # 이번에 페인트 칠을 하는거라면, 페인트 칠해야하는 구간인 L1만큼 앞으로 나간다
                    i += dashed_step_L1
                else: 
                    # 페인트 칠을 하지 않고 건너뛰는 거면, L2만큼 앞으로 나간다
                    i += dashed_step_L2


            """STEP #03 위에서 생성한 vertex로 face를 만들기 위한 vertex 조합을 생성한다"""
            if face_cnt > 0: # 첫번째 loop에는 하지 않기 위함
                if self.lane_shape[0] == 'Solid' or paint:
                    # face_cnt = 1 일 때, [0, 1, 3, 2]
                    # face_cnt = 2 일 때, [2, 3, 5, 4]
                    start_id = (face_cnt - 1) * 2
                    face = [start_id, start_id+1, start_id+3, start_id+2]

                    self.mesh_gen_vertex_subsets_for_each_face.append(face)

            face_cnt += 1 # face_cnt는 실제 paint를 칠한 것 여부와 관계없이 증가
            paint = not paint # paint 칠했으면 다음번엔 칠하지 않아야하므로 토글

        return self.mesh_gen_vertices, self.mesh_gen_vertex_subsets_for_each_face
        

    def create_mesh_gen_points_double_lane(self, solid_line_interval=0.5):
        """일정 간격으로 mesh gen point를 생성한다. 이 때, 본 클래스의 point는 0.1m 간격으로 채워져있어야한다.
        mesh gen point란, points를 기준으로, mesh를 생성할 수 있는 point를 차선을 기준으로 위 아래에 생성하는 것이다.
  
        [double lane이면]

        (0) <-- 0.5m --> (4) <-- 0.5m --> (8)

        (1) <-- 0.5m --> (5) <-- 0.5m --> (9)
        --------------------------------------
        (2)              (6)              (10)   

        (3)              (7)              (11)   
        """
        self.mesh_gen_vertices = []
        self.mesh_gen_vertex_subsets_for_each_face = []

        # 
        point_step = 0.1

        # 실선일 때
        solid_only_step = int(solid_line_interval/point_step)

        below_exec = False

        if 'Broken' in self.lane_shape:
            # 점선일 때는 다음의 계산이 필요
            dash_interval_L1 = self.dash_interval_L1
            dash_interval_L2 = self.dash_interval_L2
            L1L2 = dash_interval_L1 + dash_interval_L2
            if L1L2 == 0.0:
                Logger.log_error('lane marking init has problem')
                return self.mesh_gen_vertices, self.mesh_gen_vertex_subsets_for_each_face

            # 남는 길이를 계산하고, 이를 다시 적당히 분배해야 한다 (dashed line일 때만 계산한다)
            total_length = len(self.points) * point_step
            total_num_set = int(np.floor(total_length / L1L2))
            if total_num_set == 0: # delta 계산을 위해 total_num_set으로 나눠야하므로 계산 필요
                Logger.log_warning('lane: {} is too short to create a mesh (lane code: {}, total len: {:.2f}, L1: {}, L2: {})'.format(self.idx, self.lane_code, total_length, dash_interval_L1, dash_interval_L2))
                return self.mesh_gen_vertices, self.mesh_gen_vertex_subsets_for_each_face
            remainder = total_length % L1L2
            delta = remainder / (total_num_set * 2)
            

            # 이만큼을 각 점선 길이에 더해주면 된다
            dash_interval_L1 += delta
            dash_interval_L2 += delta

            # 이를 point index 단위로 바꾸면 아래와 같다 (ceil, floor를 일부러 교차함)
            dashed_step_L1 = int(np.ceil(dash_interval_L1/point_step))
            dashed_step_L2 = int(np.floor(dash_interval_L2/point_step))

            below_exec = True



        i = 0
        face_cnt = 0
        paint = True # 점선일 때, 그리는 구간 / 넘어가는 구간을 토글링하기 위한 변수
        
        while i < len(self.points):

            """STEP #01 vertex 생성을 위해 현재 lane marking에서 수직한 벡터 계산"""
            # 현재 위치에서의 벡터 계산
            if i == len(self.points) - 1: # 마지막 값이면, 이전 값과의 차이로 계산
                vect = self.points[i] - self.points[i-1]
            else:
                vect = self.points[i+1] - self.points[i]
            vect = vect / np.linalg.norm(vect, ord=2)

            # 현재 벡터를 xy평면상에서 +-90deg 회전시킨다.
            pos_vect_ccw = self.rorate_around_z_axis(np.pi/2, vect)
            pos_vect_cw = self.rorate_around_z_axis(-np.pi/2, vect)
            
            # 여기서 z값은 그냥 0으로 한다
            pos_vect_ccw[2] = 0
            pos_vect_cw[2] = 0


            """STEP #02 Mesh 생성을 위한 vertex 생성하기"""
            # 위 아래로 width/2 위/아래로 움직이면 된다.
            pos_vect_ccw1 = pos_vect_ccw * (self.double_line_interval / 2.0 + self.lane_width)
            pos_vect_ccw2 = pos_vect_ccw * (self.double_line_interval / 2.0)
            pos_vect_cw1 = pos_vect_cw * (self.double_line_interval / 2.0)
            pos_vect_cw2 = pos_vect_cw * (self.double_line_interval / 2.0 + self.lane_width)

            # 이를 현재 위치에서 더해주면 된다. 이 때 numpy array가 아닌 일반 list로 저장한다 (vtk에서 numpy array 지원X)
            up1 = (self.points[i] + pos_vect_ccw1).tolist()
            up2 = (self.points[i] + pos_vect_ccw2).tolist()
            dn1 = (self.points[i] + pos_vect_cw1).tolist()
            dn2 = (self.points[i] + pos_vect_cw2).tolist()

            self.mesh_gen_vertices.append(up1)
            self.mesh_gen_vertices.append(up2)
            self.mesh_gen_vertices.append(dn1)
            self.mesh_gen_vertices.append(dn2)

            if 'Broken' not in self.lane_shape:
                """ 모두 실선인 경우"""
                i += solid_only_step
            else:
                if not below_exec:
                    Logger.log_debug('WTF')
                """ 점선이 섞여있는 경우"""
                if paint:
                    # 이번에 페인트 칠을 하는거라면, 페인트 칠해야하는 구간인 L1만큼 앞으로 나간다
                    i += dashed_step_L1
                else: 
                    # 페인트 칠을 하지 않고 건너뛰는 거면, L2만큼 앞으로 나간다
                    i += dashed_step_L2


            """STEP #03 위에서 생성한 vertex로 face를 만들기 위한 vertex 조합을 생성한다"""
            if face_cnt > 0: # 첫번째 loop에는 하지 않기 위함

                # face_cnt = 1 일 때, [0, 1, 5, 4], [2, 3, 7, 6]
                # face_cnt = 2 일 때, [4, 5, 9, 8], [6, 7, 11, 10]

                start_id = (face_cnt - 1) * 4
                face1 = [start_id+0, start_id+1, start_id+5, start_id+4]
                face2 = [start_id+2, start_id+3, start_id+7, start_id+6]

                if self.lane_shape[0] == 'Solid' or paint: # solid 일 때는 항상 paint = True이므로, lane_shape는 신경쓰지 않아도 된다.
                    self.mesh_gen_vertex_subsets_for_each_face.append(face1)

                if self.lane_shape[1] == 'Solid' or paint:
                    self.mesh_gen_vertex_subsets_for_each_face.append(face2)

            face_cnt += 1 # face_cnt는 실제 paint를 칠한 것 여부와 관계없이 증가
            paint = not paint # paint 칠했으면 다음번엔 칠하지 않아야하므로 토글
            
        return self.mesh_gen_vertices, self.mesh_gen_vertex_subsets_for_each_face
            
            
    def rorate_around_z_axis(self, angle, point):
        rotation = np.array([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle),  np.cos(angle), 0.0],
            [0.0, 0.0, 1.0]])

        transform_pt = rotation.dot(point)
        return transform_pt

    
    def draw_plot(self, axes):
        # if self.get_lane_num() != 2:
        #     return
        # if self.lane_code != 502:
        #     return
            
        # 그려야하는 width와 color가 지정되어 있으면 해당 값으로만 그린다
        if self.vis_mode_line_width is not None and \
            self.vis_mode_line_color is not None:
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                linewidth=self.vis_mode_line_width,
                color=self.vis_mode_line_color,
                markersize=self.vis_mode_marker_size,
                marker=self.vis_mode_marker_style)
            return
        

        if self.get_vis_mode_all_different_color():
            # 모두 다르게 그리라고하면, 색을 명시하지 않으면 된다
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                markersize=self.vis_mode_marker_size,
                marker=self.vis_mode_marker_style)
        
        else:
            # 이 경우에는 선의 종류에 따라 정해진 색과 모양으로 그린다
            
            if not self.included_plane:
                # 이는 list of matplotlib.lines.Line2D 인스턴스를 반환
                self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                    markersize=self.vis_mode_marker_size,
                    marker=self.vis_mode_marker_style,
                    color='k')
            else:
                self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                    markersize=self.vis_mode_marker_size,
                    marker=self.vis_mode_marker_style,
                    color='b')

        


    @staticmethod
    def from_dict(dict_data, node_set=None):
        idx = dict_data['idx']
        from_node_idx = dict_data['from_node_idx']
        to_node_idx = dict_data['to_node_idx']
        points = np.array(dict_data['points'])
        
        """이제 node와 연결해준다"""
        if node_set is not None:
            start_node = node_set.nodes[from_node_idx]
            end_node = node_set.nodes[to_node_idx]

        lane_marking = LaneMarking(points=points, idx=idx)
        lane_marking.set_from_node(start_node)
        lane_marking.set_to_node(end_node)

        lane_marking.lane_code_def = dict_data['lane_code_def']
        lane_marking.lane_code = dict_data['lane_code']
        lane_marking.lane_color = dict_data['lane_color']
        lane_marking.lane_shape = dict_data['lane_shape']
        lane_marking.lane_width = dict_data['lane_width']
        lane_marking.dash_interval_L1 = dict_data['dash_interval_L1']
        lane_marking.dash_interval_L2 = dict_data['dash_interval_L2']
        lane_marking.double_line_interval = dict_data['double_line_interval']

        return lane_marking

    
    @staticmethod
    def copy_attribute(src, dst):
        dst.lane_code_def = src.lane_code_def
        dst.lane_code = src.lane_code

        dst.lane_color = src.lane_color
        dst.lane_shape = src.lane_shape

        # dash solid_line_interval
        dst.lane_width = src.lane_width
        dst.dash_interval_L1 = src.dash_interval_L1
        dst.dash_interval_L2 = src.dash_interval_L2
        dst.double_line_interval = src.double_line_interval


    def item_prop(self):
        item = self.to_dict()
        prop_data = OrderedDict()

        prop_data['idx'] = {'type' : 'string', 'value' : item['idx']}
        prop_data['points'] = {'type' : 'list<list<float>>', 'value' : item['points']}
        prop_data['lane_code'] = {'type' : 'int', 'value' : item['lane_code']}
        prop_data['lane_code_def'] = {'type' : 'string', 'value' : item['lane_code_def']}
        prop_data['lane_color'] = {'type' : 'string', 'value' : item['lane_color']}
        prop_data['lane_shape'] = {'type' : 'string', 'value' : item['lane_shape']}
        prop_data['lane_width'] = {'type' : 'float', 'value' : item['lane_width']}
        prop_data['dash_interval_L1'] = {'type' : 'float', 'value' : item['dash_interval_L1']}
        prop_data['dash_interval_L2'] = {'type' : 'float', 'value' : item['dash_interval_L2']}
        prop_data['double_line_interval'] = {'type' : 'float', 'value' : item['double_line_interval']}

        return prop_data
