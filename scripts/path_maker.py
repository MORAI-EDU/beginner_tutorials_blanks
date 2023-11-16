#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt
from turtlesim.msg import Pose


class pathMaker :
    
    def __init__(self, pkg_name, path_name):
        rospy.init_node('path_maker', anonymous=True)
        # /turtle1/pose 토픽 구독
        rospy.Subscriber("/turtle1/pose", Pose, self.status_callback)
        # 초기화
        self.prev_x = 0
        self.prev_y = 0
        self.is_status=False
        # 패키지 경로 로드 & 파일 쓰기 모드
        rospack = rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        full_path=pkg_path + '/'+path_name+'.txt'
        self.f=open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_status==True :
                # turtle 위치 기록
                self.path_make()
        self.f.close()

    def path_make(self):
        x=self.status_msg.x
        y=self.status_msg.y
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        # 이전 waypoint와의 거리가 0.3 이상이어야 기록
        if distance >0.3:
            data='{0}\t{1}\n'.format(x,y)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            print("write : ", x,y)
    
    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg

if __name__ == '__main__' :
    try:
        p_m=pathMaker("beginner_tutorials", "turtle_path")
    except rospy.ROSInternalException:
        pass
            