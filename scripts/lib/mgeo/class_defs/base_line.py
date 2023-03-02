#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from utils.logger import Logger

import numpy as np
import math

class BaseLine(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, _points=None, idx=None):
        self.points = None
        self.idx = idx  # Line이 속한 LineSet 인스턴스에서의 index이다.

        # 각 Line의 bbox
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
            raise BaseException('[ERROR] @ BaseLine.set_points: _points must be an instance of numpy.ndarray of list. Type of your input = {}'.format(type(_points)))
    
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


    def decimate_points(self, decimation):
        _indx_del = list()
        for i in range(len(self.points)):
            if (i % decimation) != 0:
                _indx_del.append(i)
            if i == len(self.points) - 1:
                _indx_del.pop()
        
        # np.delete는 기존 ndarray의 복사본을 return한다
        _decimated_array = np.delete(self.points, _indx_del, 0)
        self.points = _decimated_array


    def get_num_points(self):
        return self.points.shape[0]


    def get_total_distance(self):
        total_distance = 0
        for i in range(len(self.points) - 1) :
            vect = self.points[i+1] - self.points[i]
            dist_between_each_point_pair = np.linalg.norm(vect, ord=2)
            total_distance += dist_between_each_point_pair
        return total_distance


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


    def create_the_first_point(self, first_point):
        '''
        첫번째 점을 설정한다. 내부적으로 points는 np.ndarray 타입으로, [ [1,2,3], [4,5,6], [7,8,9], ... ] 형식으로 저장되므로
        넘겨준 좌표를 이용하여 points가 np.array([[x0, y0, z0]])로 저장되도록 한다/
        :param first_point: 첫번째 점으로, [1,2,3], [[1,2,3]], np.array([1,2,3]), np.array([[1,2,3]]) 의 형태가 되어야 한다
        '''
        if type(first_point) is np.ndarray:
            if first_point.shape == (1,3):
                self.set_points(first_point)
            
            elif first_point.shape == (3, ):
                self.set_points(np.array([first_point]))
            else:
                raise BaseException('[ERROR] first_point argument must be one of these: [1,2,3], [[1,2,3]], np.array([1,2,3]), np.array([[1,2,3]])')

        elif type(first_point) is list:
            first_point = np.array(first_point)
            self.create_the_first_point(first_point)

        else:
            raise BaseException('[ERROR] first_point argument must be one of these: [1,2,3], [[1,2,3]], np.array([1,2,3]), np.array([[1,2,3]])')


    def create_points_from_current_pos_using_step(self, xyz_step_size, step_num):
        '''
        현재 points의 마지막 지점으로부터 points를 생성해나간다
        '''
        if self.points is None: # [NOTE] numpy array일 경우 == None으로 테스트하면 안된다! 그럼 numpy array의 각 값이 None인지 확인하기 때문
            raise BaseException('[ERROR] initialize the first point first.')
        current_pos = self.points[-1]

        new_points = self._create_points_using_step(
            current_pos, xyz_step_size, step_num)

        self.set_points(np.vstack((self.points, new_points)))


    def add_new_points(self, points_to_add):
        '''
        현재 있는 points에 점을 추가한다
        '''
        self.set_points(np.vstack((self.points, points_to_add)))


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


    def fill_in_points_evenly_accor_to_leng(self, step_len):
        new_points = self.calculate_evenly_spaced_link_points_accor_to_leng(step_len)
        self.set_points(new_points)


    def calculate_evenly_spaced_link_points_accor_to_leng(self, step_len):
        '''
        현재의 링크를 일정한 간격으로 채워주는 점의 집합을 계산한다
        
        ex) line_total_length = 32, step_len = 5
        line_total_length / step_len = 6.4
        *----*----*----*----*----*----*-*
                        ↓
        >> new_step_len = 4
        line_total_length / new_step_len = 8
        *---*---*---*---*---*---*---*---*

        '''
        total_dist = 0
        new_step_len = step_len
        for i in range(len(self.points) - 1):
            point_now  = self.points[i]
            point_next  = self.points[i+1]
            dist_point = np.array(point_next) - np.array(point_now)
            dist = math.sqrt(pow(dist_point[0], 2) + pow(dist_point[1], 2) + pow(dist_point[2], 2))
            total_dist += dist
            if 0 < total_dist//step_len < step_len/2:
                new_step_len = total_dist/(total_dist//step_len)
            else:
                new_step_len = total_dist/(total_dist//step_len+1)
        new_points_all = self.points[0]
        print(step_len, new_step_len, total_dist)

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
            step_vect = new_step_len * unit_vect
            
            # 2.2) 벡터를 몇 번 전진할 것인가
            cnt = (int)(np.floor(mag / new_step_len))
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
            if mag % new_step_len == 0:
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