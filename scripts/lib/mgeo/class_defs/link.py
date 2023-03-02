#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from utils.logger import Logger

from class_defs.line import Line
import numpy as np 

from collections import OrderedDict

class Link(Line):
    '''
    내부의 points 필드를 처음부터 초기화하지 않고 나중에 만들 수 있는 클래스이다.
    
    lazy_point_init 필드가 True이면, point 변수 초기화를 나중에 할 수 있다.
    이는 차선 변경이 가능함을 표현하기 위한 클래스로, 아래 예시를 통해 정의를 이해할 수 있다.

    아래와 같이 편도 2차선인 도로를 가정하면 도로의 양끝에는 노드가 2개씩 있어,
    총 4개의 노드가 정의된다.
    
    예제)
    ======> 실제 도로 방향 =====>
    Node1A                Node2A
    Node1B                Node2B

    이 때 어느 쪽으로든 차선이 변경 가능하다고 하면, 총 4종류의 링크가 생성 가능한데,
    
    Node1A -> Node2A
    Node1B -> Node2B
    위 2가지 링크는 차선 변경을 하지 않는 링크로, 
      실제 차가 따라가야할 경로가 fix되어 있는 셈이다.
      이 경우 lazy_point_init = False로 정의하고, points 필드에 경로점이 정의되어 있다.
    
    Node1A -> Node2B
    Node1B -> Node2A
    위 2가지 링크는 차선 변경을 하는 링크로,
      실제 차가 따라가야할 경로는 고정되어 있지 않다 (차선 변경을 어느 시점에든 할 수 있으므로)
      이 경우 lazy_point_init = True로 정의하고, points 필드는 연결해야하는 양 끝점만 가지고 있다.

    '''
    def __init__(self, points=None, idx=None, lazy_point_init=False, link_type=None, road_type=None):        
        self.lazy_point_init = lazy_point_init
        super(Link, self).__init__(points, idx)

        # 차선 변경이 아닐 경우 이 값이 유효. 차선 변경 링크를 생성하기 위한 값들
        self.lane_ch_link_left = None # 좌측 차선 진입으로 들어갈 수 있는 링크
        self.lane_ch_link_right = None # 우측 차선 진입으로 들어갈 수 있는 링크

        # 같은 도로 소속인 차선의 관계성 확보용 변수
        self.lane_group = None

        # 차선 변경일 경우
        self.lane_change_pair_list = list()

        # 최대 속도 및 최저 속도
        self.max_speed_kph = 0
        self.min_speed_kph = 0

        self.link_type = link_type
        self.road_type = road_type

        # 해당 링크에 연결된 object들
        self.traffic_signs = list()
        self.traffic_lights = list()
        self.surface_markings = list()

        # 42dot 데이터
        self.road_id = ''
        self.ego_lane = None
        self.lane_change_dir = None
        self.hov = None

        # stryx 데이터
        self.related_signal = None        
        self.its_link_id = None
        self.can_move_left_lane = False
        self.can_move_right_lane = False
        self.road_type = None
     
        # OpenDRIVE 생성 관련 >> width 강제 설정
        fw, ws, fe, we = self.get_default_width_related_values()
        self.force_width_start = fw
        self.width_start = ws
        self.force_width_end = fe
        self.width_end = we

        # OpenDRIVE 생성 관련 >> sidewalk 설정
        self.enable_side_border = False
        # self.width_start = -1 # -1 이면 b.c.에 따라 자동 설정된다
        # self.width_end = -1 # -1 이면 b.c.에 따라 자동 설정된다

        # OpenDRIVE 생성 관련 >> ref line 찾을 때 사용
        # get_max_succeeding_links를 dynamic programming으로 풀기위한 변수
        self.reset_odr_conv_variables()

        self.geometry = [{'id':0, 'method':'poly3'}]

    def set_points(self, points):
        super(Link, self).set_points(points)
        # NOTE: cost 계산 시 고려되어야 하는 부분이 너무 많아서, 이를 set_points에 묶어둘 수 없다.
        # self.calculate_cost()
    
    def is_it_for_lane_change(self):
        return self.lazy_point_init

    def get_traffic_signs(self):
        return self.traffic_signs

    def get_traffic_lights(self):
        return self.traffic_lights

    def get_surface_markings(self):
        return self.surface_markings
    
    def add_geometry(self, point_id, method):
        if point_id == len(self.points) - 1:
            raise BaseException('adding geometry point in the last point is not supported.')

        # 만약 현재 point_id 가 이미 있으면 현재 입력된 method로 변경하고 리턴한다
        for geo_point in self.geometry:
            if geo_point['id'] == point_id:
                geo_point['method'] = method
                return

        # 이 경우는 현재 전달된 point_id가 self.geometry 내부에 없는 경우가 된다.
        # 이 point_id를 추가해주고, sort 해주면 된다.
        self.geometry.append({'id':point_id, 'method':method})

        # 추가할때마다 'id'를 기준으로 ascending-order로 sort해준다
        self.geometry = sorted(self.geometry, key=lambda element: element['id'])


    ''' 차선 변경으로 진입 가능한 링크 설정 ''' 
    def set_left_lane_change_dst_link(self, link):
        if type(link).__name__ != 'Link':
            raise BaseException('[ERROR] unexpected link type: {}'.format(type(link)))    
        self.lane_ch_link_left = link

    def set_right_lane_change_dst_link(self, link):
        if type(link).__name__ != 'Link':
            raise BaseException('[ERROR] unexpected link type: {}'.format(type(link)))        
        self.lane_ch_link_right = link

    def get_left_lane_change_dst_link(self):
        if self.is_it_for_lane_change():
            raise BaseException('[ERROR] lane_change_dst_link is only defined when self.is_it_for_lane_change() == False')
        return self.lane_ch_link_left

    def get_right_lane_change_dst_link(self):
        if self.is_it_for_lane_change():
            raise BaseException('[ERROR] lane_change_dst_link is only defined when self.is_it_for_lane_change() == False')
        return self.lane_ch_link_right
    

    ''' 차선 변경 관련'''
    def get_lane_change_pair_list(self):
        return self.lane_change_pair_list
    
    def set_lane_change_pair_list(self, info):
        self.lane_change_pair_list = info

    def get_number_of_lane_change(self):
        if not self.is_it_for_lane_change():
            return 0
        else:
            return len(self.lane_change_pair_list)


    def get_all_left_links(self, check_road=True):
        """좌측 차선 변경으로 진입할 수 있는 모든 링크 리스트를 반환한다.
        check_road는 True이면, 현재 링크와 road가 같은 lane_ch_link_left 중에서 찾는다. (즉 road가 다른 link가 나타나면 중단)
        """
        ret_list = list()

        current_link = self
        left_link = current_link.lane_ch_link_left
        while left_link is not None:
            # link 오류로 인해, ret_list에서 다시 left_link가 검출되었다면 오류이다.
            if left_link in ret_list:
                raise BaseException('link: {} has a logical error. get_all_left_lanes detected an infinite-loop.'.format(current_link.idx))
            
            # road_id를 체크하는 경우라면,
            if check_road:
                # road_id가 다른 link가 발견되면 종료한다
                if left_link.road_id != current_link.road_id:
                    break

            ret_list.append(left_link)
            
            # 현재 링크를 다시 left_link로 업데이트하고, left_link 또한 업데이트
            current_link = left_link
            left_link = current_link.lane_ch_link_left

        return ret_list
    
    
    def get_all_right_links(self, check_road=True):
        """우측 차선 변경으로 진입할 수 있는 모든 링크 리스트를 반환한다.
        check_road는 True이면, 현재 링크와 road가 같은 lane_ch_link_right 중에서 찾는다. (즉 road가 다른 link가 나타나면 중단)
        """
        ret_list = list()

        current_link = self
        right_link = current_link.lane_ch_link_right
        while right_link is not None:
            # link 오류로 인해, ret_list에서 다시 left_link가 검출되었다면 오류이다.
            if right_link in ret_list:
                raise BaseException('link: {} has a logical error. get_all_right_links detected an infinite-loop.'.format(current_link.idx))
            
            # road_id를 체크하는 경우라면,
            if check_road:
                # road_id가 다른 link가 발견되면 종료한다
                if right_link.road_id != current_link.road_id:
                    break

            ret_list.append(right_link)
            
            # 현재 링크를 다시 right_link로 업데이트하고, right_link 또한 업데이트
            current_link = right_link
            right_link = current_link.lane_ch_link_right

        return ret_list


    def is_in_the_left_or_right_side(self, another_link):
        """현재 링크가 another_link의 왼쪽 또는 오른쪽에 있는지 찾아준다. 왼쪽/오른쪽 어디에도 없으면 False, ''가 반환된다"""
        if self in another_link.get_all_left_links():
            return True, 'left'

        elif self in another_link.get_all_right_links():
            return True, 'right'

        else:
            return False, ''


    """ OpenDRIVE 변환 관련 """
    def reset_odr_conv_variables(self):
        """
        OpenDRIVE 변경 관련 기능 중, 프로그램 실행 중에 계속 초기화가 필요한 변수가 있다
        해당 변수를 초기화하는 함수
        """
        self.odr_lane = None

        # 아래는 refernece line 찾는데 사용하는 알고리즘용 변수
        self.max_succeeding_link_solution_calculated = False
        self.max_succeeding_link_solution = (1, [self])


    def get_max_succeeding_links_within_the_same_road(self, road_id=None, preceding_links=None):
        """
        현재 road 내에서, 현재 링크로부터 가장 긴 (link 수가 많은) 다음 링크의 리스트를 구한다
        """
        # 이미 해당 인스턴스에 대해서는 솔루션이 구해졌으면 이를 반환한다.
        if self.max_succeeding_link_solution_calculated:
            return self.max_succeeding_link_solution

        # 처음 호출 시 초기화를 위한 값이다. recursive call 때는 road_id를 입력해주어야 한다
        if road_id is None:
            road_id = self.road_id

        # road를 closed-loop인 형태로 정의한 경우, 이를 오류로 인식하도록 한다. (이를 처리하지 않으면 recursive call이 무한히 반복됨)
        # recursive call이 무한히 반복되는 오류를 미리 피하기 위해, preceding_links를 전달해야하며, 이를 초기화한다
        if preceding_links is None:
            preceding_links = (self, ) # 초기화 때임. 반드시 tuple로 해야하며, 1개 item의 tuple 초기화는 (item, )으로 한다
            # 왜 반드시 tuple 이어야하는지는 아래에서 설명
        else:
            # 만일 현재 링크가 preceding link에 존재한다면, 현재 road가 closed-loop인 형태인 것이다.
            if self in preceding_links:
                raise BaseException('Error found in Link: {} >> Road: {} has a logical error: It is a circular (closed-loop) road, which is not allowed.'.format(self.idx, road_id))

            preceding_links = preceding_links + (self, )

        # 이제 이 최적문제를 recursive call로 푼다
        max_value = 1
        solution_link_list = [self]
        for to_link in self.get_to_links():
            # 같은 road_id가 아니면 무시한다
            if to_link.road_id != road_id:
                continue

            # 같은 road_id이면, 이 링크에 대해 다시 subproblem을 푼다
            
            # NOTE: 이 때 무한 루프 방지를 위해 preceding_links를 넘겨주어야 하고, 이 때 반드시 tuple을 써야 한다.
            # preceding_links_for_this = preceding_links + (to_link, ) 
            # 위와 같이, to_links는 계속 반복되면서 변경되는 값으로, to_link에 따라 preceding_links가 변경되어 recursive call (아래쪽에) 전달해야한다
            # 그런데, 리스트로 위와 같은 동작을 하려면 번거롭다. 
            # preceding_links_for_this = preceding_links.append(to_link) # 리스트로는 이렇게 불가능하다!! preceding_links 자체가 업데이트 되어버리므로.

            # print('link: {}, preceding_links_for_this: {}'.format(self.idx, Link.get_id_list_string(preceding_links_for_this)))
            sub_solution, sub_link_list = to_link.get_max_succeeding_links_within_the_same_road(road_id, preceding_links)

            # 그러면 현재 link에 대해, 솔루션은 아래와 같다.
            num = sub_solution + 1

            # 이 값이 현재 저장된 값보다 크면, 저장된 값을 업데이트한다
            if num > max_value:
                max_value = num
                solution_link_list = [self] + sub_link_list

        # 다시 이 함수가 호출되었을 때에는 새로 계산하지 않고 저장된 값을 전달할 수 있게 솔루션을 저장해둔다.
        self.max_succeeding_link_solution_calculated = True
        self.max_succeeding_link_solution = (max_value, solution_link_list)

        return self.max_succeeding_link_solution

    """ OpenDRIVE 변환 관련 (끝)"""


    def set_values_for_lane_change_link(self, lane_change_path):
        '''
        본 링크가 차선 변경을 표현하고자하는 링크일 때, 
        lane_change_path = [A, B, C, D] 와 같은 식으로 넣어주면 된다. 
        - from_node는 A의 from_node,
          to_node  는 D의 to_node,
        - lane_change_pair_list는 [from A -> to B], [from B -> to C], [from C -> to D]
        '''
        if not self.lazy_point_init:
            raise BaseException('lazy_point_init is True => USE Line.set_points_using_node instead of this!! (cannot use set_points_using_node_lazy_init)')
        
        if len(lane_change_path) < 2:
            raise BaseException('len(lane_change_path) must be >= 2 !! length of the current input = {}'.format(len(lane_change_path)))

        # from node, to node 설정 
        from_node = lane_change_path[0].get_from_node()
        to_node = lane_change_path[-1].get_to_node()
        # NOTE: 이미 해당 링크가 먼저 파일등으로부터 로드되었다면, from_node, to_node 등은 미리 설정되어있을 것이다.
        # 전달된 데이터의 오류로 링크의 노드 정보가 없을 수 있기 때문에 예외처리 코드 추가
        if from_node is None or to_node is None:
            return

        # NOTE : https://morai.atlassian.net/browse/MS-62
        # self.set_from_node(from_node)
        # self.set_to_node(to_node)

        # points 설정
        p1 = from_node.point
        p2 = to_node.point
        points = p1
        points = np.vstack((points, p2))
        self.set_points(points)

        # 
        lane_change_pair_list = [] 
        for i in range(len(lane_change_path) - 1):
           lane_change_pair_list.append({'from': lane_change_path[i], 'to': lane_change_path[i+1]})
        self.set_lane_change_pair_list(lane_change_pair_list)

    def set_max_speed_kph(self, max_speed_kph):
        self.max_speed_kph = max_speed_kph

    def set_min_speed_kph(self, min_speed_kph):
        self.min_speed_kph = min_speed_kph

    def get_max_speed_kph(self):
        return self.max_speed_kph

    def get_min_speed_kph(self):
        return self.min_speed_kph


    def set_width(self, width):
        if width is None:
            return
        self.width = width
        # self.offset = self.width/2

    def set_width_related_values(self, force_width_start, width_start, force_width_end, width_end):
        self.force_width_start = force_width_start
        self.width_start = width_start
        self.force_width_end = force_width_end
        self.width_end = width_end

    def get_width(self):
        return self.width

    def get_offset(self):
        return self.offset

    def calculate_cost(self):
        '''
        points 필드를 바탕으로, cost를 계산한다.
        set_points가 초기화코드에서 호출되면서 point가 설정이 안 된 채로 호출될 수 있는데,
        이 때는 그냥 리턴한다. (TODO: 향후 코드 개선 필요.
        이건 사실 근본적으로 Line쪽의 문제임. ctor에서는 set_points를 호출하지 않든지 해야 함)
        '''
        if self.points is None:
            # Logger.log_warning('calculate_cost is called without points attribute initialized')
            return 
        

        # 거리 계산
        # TODO: 해당 차로의 속도를 생각해서, 시간을 기준으로 고려할 것
        if self.is_it_for_lane_change():
            # 변경해서 들어갈 마지막 차선의 distance로 계산한다
            # NOTE: 중요한 가정이 있음. 차선 변경 진입 전후의 링크 길이가 거의 같아야 한다
            # 차선 변경 진입 후 링크가 너무 길다거나 하면 차선 변경 링크 생성 이전에 편집이 필요

            lane_ch_pair_list = self.get_lane_change_pair_list()
            last_to_link = lane_ch_pair_list[-1]['to']
            distance = last_to_link.get_total_distance()
        else:
            distance = self.get_total_distance()


        # 차선 변경에 따른 cost 계산
        # TODO: 해당 차로의 속도를 생각해서, 차선 변경 가능 시간을 기준으로 고려할 것
        def calc_lane_change_cost(x):  
            # 기준이 되는 값
            x_org = [10, 50, 100, 500, 1000, 2000]
            y_org = [500, 300, 200,  50, 20, 10]
            return np.interp(x, x_org, y_org, left=float('inf'), right=y_org[-1])

        lane_ch_pair_list = self.get_lane_change_pair_list()
        if self.is_it_for_lane_change():
            # 차선 변경이 3번이면, 전체 링크 길이를 L이라 할 때
            # L/3 인 차선 변경 cost를 계산한 다음, 3을 곱하여 전체 차선 변경 penalty를 계산
            lc_num = self.get_number_of_lane_change()
            unit_distance = distance / lc_num
            lane_change_penalty = lc_num * calc_lane_change_cost(unit_distance)
        else:
            lane_change_penalty = 0            

        self.cost = distance + lane_change_penalty
          
    def draw_plot(self, axes):

        # 그려야하는 width와 color가 지정되어 있으면 해당 값으로만 그린다
        if self.vis_mode_line_width is not None and \
            self.vis_mode_line_color is not None:
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                linewidth=self.vis_mode_line_width,
                color=self.vis_mode_line_color,
                markersize=2,
                marker='o')
            return
            
        if self.get_vis_mode_all_different_color():
            # 모두 다르게 그리라고하면, 색을 명시하지 않으면 된다
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                markersize=2,
                marker='o')
                
        else:
            # 이 경우에는 선의 종류에 따라 정해진 색과 모양으로 그린다
            if not self.lazy_point_init:
                self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                    linewidth=1,
                    markersize=2,
                    marker='o',
                    color='k')
            else:
                self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                    linewidth=1,
                    markersize=2,
                    marker='o',
                    color='b')


    @staticmethod
    def copy_attributes(src, dst):
        dst.lane_group = src.lane_group
        dst.lane_change_pair_list = src.lane_change_pair_list
        
        
        dst.max_speed_kph = src.max_speed_kph
        dst.min_speed_kph = src.min_speed_kph
        dst.link_type = src.link_type
        
        
        dst.road_id = src.road_id
        dst.ego_lane = src.ego_lane
        dst.lane_change_dir = src.lane_change_dir
        dst.hov = src.hov


    def is_dangling_link(self):
        if self.from_node is None or self.to_node is None:
            return True
        else:
            return False


    def has_location_error_node(self):
        sp_distance = 0.0
        ep_distance = 0.0

        if self.from_node:
            pos_vect = self.points[0] - self.from_node.point
            sp_distance = np.linalg.norm(pos_vect)

        if self.to_node:
            pos_vect = self.points[len(self.points) - 1] - self.to_node.point
            ep_distance = np.linalg.norm(pos_vect)

        if sp_distance < 1.0 and ep_distance < 1.0:
            return False
        else:
            return True


    def to_dict(self):
        """json 파일 등으로 저장할 수 있는 dict 데이터로 변경한다"""
        
        # 차선 변경으로 진입 가능한 차선 정보    
        if not self.is_it_for_lane_change():
            # 일반 링크이면
            if self.get_left_lane_change_dst_link() is None:
                left_lane_change_dst_link_idx = None
            else:
                left_lane_change_dst_link_idx = self.get_left_lane_change_dst_link().idx
            
            if self.get_right_lane_change_dst_link() is None:
                right_lane_change_dst_link_idx = None
            else:
                right_lane_change_dst_link_idx = self.get_right_lane_change_dst_link().idx
        else:
            # 차선 변경 링크이면
            left_lane_change_dst_link_idx = None
            right_lane_change_dst_link_idx = None


        # 차선 변경 링크인 경우, 차선 변경 Path 
        lane_ch_link_path = []
        pair_list = self.get_lane_change_pair_list()
        for i in range(len(pair_list)):
            pair = pair_list[i]

            lane_ch_link_path.append(pair['from'].idx)

            # 마지막이면, 
            if (i == len(pair_list) - 1):
                lane_ch_link_path.append(pair['to'].idx)

        dict_data = {
            'idx': self.idx,
            'from_node_idx': self.from_node.idx if self.from_node else None,
            'to_node_idx': self.to_node.idx if self.to_node else None,
            'points': self.points.tolist(),
            'max_speed': self.max_speed_kph,
            'lazy_init': self.lazy_point_init,
            'can_move_left_lane': self.can_move_left_lane,
            'can_move_right_lane': self.can_move_right_lane,
            'left_lane_change_dst_link_idx': left_lane_change_dst_link_idx,
            'right_lane_change_dst_link_idx': right_lane_change_dst_link_idx, 
            'lane_ch_link_path': lane_ch_link_path,
            'link_type': self.link_type,
            'road_type': self.road_type,
            'road_id': self.road_id,
            'ego_lane': self.ego_lane,
            'lane_change_dir': self.lane_change_dir,
            'hov': self.hov,
            'geometry': self.geometry,         
            'related_signal': self.related_signal,
            'its_link_id': self.its_link_id,  
            'force_width_start': self.force_width_start,
            'width_start': self.width_start,
            'force_width_end': self.force_width_end,
            'width_end': self.width_end,
            'enable_side_border': self.enable_side_border
        }
        return dict_data


    @staticmethod
    def from_dict(dict_data, link_set=None):
        pass

        
    @classmethod
    def get_id_list_string(cls, list_obj):
        ret_str = '['
        for obj in list_obj:
            ret_str += '{}, '.format(obj.idx)
        ret_str += ']'
        ret_str = ret_str.replace(', ]', ']')
        return ret_str


    @staticmethod
    def get_default_width_related_values():
        return False, 3.5, False, 3.5


    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx}
        prop_data['points'] = {'type' : 'list<list<float>>', 'value' : self.points.tolist()}
        prop_data['from_node'] = {'type' : 'string', 'value' : self.from_node.idx if self.from_node else None}
        prop_data['to_node'] = {'type' : 'string', 'value' : self.to_node.idx if self.to_node else None}
        prop_data['can_move_left_lane'] = {'type' : 'boolean', 'value' : self.can_move_left_lane}
        prop_data['can_move_right_lane'] = {'type' : 'boolean', 'value' : self.can_move_right_lane}
        prop_data['lane_ch_link_left'] = {'type' : 'string', 'value' : self.lane_ch_link_left.idx if self.lane_ch_link_left else None}
        prop_data['lane_ch_link_right'] = {'type' : 'string', 'value' : self.lane_ch_link_right.idx if self.lane_ch_link_right else None}
        prop_data['max_speed_kph'] = {'type' : 'int', 'value' : self.get_max_speed_kph()}
        prop_data['min_speed_kph'] = {'type' : 'int', 'value' : self.get_min_speed_kph()}
        prop_data['link_type'] = {'type' : 'string', 'value' : self.link_type}
        prop_data['road_type'] = {'type' : 'string', 'value' : self.road_type}
        prop_data['road_id'] = {'type' : 'string', 'value' : self.road_id}
        prop_data['ego_lane'] = {'type' : 'int', 'value' : self.ego_lane}
        prop_data['hov'] = {'type' : 'boolean', 'value' : self.hov}
        prop_data['related_signal'] = {'type' : 'string', 'value' : self.related_signal}
        prop_data['its_link_id'] = {'type' : 'string', 'value' : self.its_link_id}
        prop_data['geometry'] = {'type' : 'list<dict>', 'value' : self.geometry}
        prop_data['force width (start)'] = {'type' : 'boolean', 'value' : self.force_width_start}
        prop_data['width_start'] = {'type' : 'float', 'value' : self.width_start}
        prop_data['force width (end)'] = {'type' : 'boolean', 'value' : self.force_width_end}
        prop_data['width_end'] = {'type' : 'float', 'value' : self.width_end}
        prop_data['side_border'] = {'type' : 'boolean', 'value' : self.enable_side_border}
        return prop_data