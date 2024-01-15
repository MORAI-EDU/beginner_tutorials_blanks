#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf
# /turtle1/pose 토픽 타입인 turtlesim/Pose import
from turtlesim.msg import Pose

class turtle_listener():
    def __init__(self):
        rospy.init_node('status_listener', anonymous=True)
        # /turtle1/pose 구독
        rospy.Subscriber('/turtle1/pose', Pose, self.statusCB)
        self.status_msg=Pose()
        rospy.spin()

    def statusCB(self,data): ## turtle Status subscriber
        self.status_msg=data
        print("tf broad cast")
        # 브로드캐스터 생성
        br = tf.TransformBroadcaster()
        # turtle1 상태 tf 브로드캐스팅
        br.sendTransform((self.status_msg.x, self.status_msg.y, 0),
                        tf.transformations.quaternion_from_euler(0,0, self.status_msg.theta),
                        rospy.Time.now(),
                        "turtle",
                        "map")

if __name__ == '__main__':
    try:
        tl=turtle_listener()
    except rospy.ROSInternalException:
        pass
