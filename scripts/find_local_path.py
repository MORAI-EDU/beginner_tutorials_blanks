#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path
import tf
from math import sqrt
from geometry_msgs.msg import PoseStamped
from path_reader import pathReader
from turtlesim.msg import Pose

# global_path 와 turtle의 status_msg를 이용해
# 현재 waypoint와 local_path를 생성
def find_local_path(ref_path,status_msg):
    out_path=Path()
    current_x=status_msg.x
    current_y=status_msg.y
    current_waypoint=0
    min_dis=float('inf')

    # 가장 가까운 waypoint(current waypoint) 찾기
    for i in range(len(ref_path.poses)):
        dx = current_x - ref_path.poses[i].pose.position.x
        dy = current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i

    # 현재 waypoint 부터 최대 10개의 waypoint를 local_path에 추가
    if current_waypoint+10 > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)
    else :
        last_local_waypoint=current_waypoint+10

    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint):
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=0
        out_path.poses.append(tmp_pose)

    return out_path,current_waypoint


class turtle_listener():
    def __init__(self):
        rospy.Subscriber('/turtle1/pose',Pose, self.statusCB)
        self.status_msg=Pose()

    def statusCB(self,data): ## turtle Status Subscriber
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.x, self.status_msg.y, 0),
                        tf.transformations.quaternion_from_euler(0,0, self.status_msg.theta),
                        rospy.Time.now(),
                        "turtle",
                        "map")

if __name__ == '__main__' :
    try:
        rospy.init_node('local_path_finder', anonymous=True)
        path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
        local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        tl=turtle_listener()
        # 전역 경로 로드
        p_r=pathReader("beginner_tutorials")
        global_path = p_r.read_txt("turtle_path.txt")

        rate=rospy.Rate(30)
        while not rospy.is_shutdown():
            # 지역 경로 생성
            local_path,current_waypoint = find_local_path(global_path, tl.status_msg)
            local_path_pub.publish(local_path)
            path_pub.publish(global_path)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
