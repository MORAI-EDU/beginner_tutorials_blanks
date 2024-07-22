#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, os
from morai_msgs.msg import CtrlCmd, CollisionData, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from enum import Enum

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4

class s_drive():
    def __init__(self):
        rospy.init_node('collision_avoid', anonymous=True)
        
        # publisher
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        
        # subscriber
        rospy.Subscriber('/CollisionData', CollisionData, self.collision_callback)
        self.is_collision_data = False
        self.is_collision = False

        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        self.is_ego = False

        # service
        rospy.wait_for_service('/Service_MoraiEventCmd', timeout= 5)

        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.rate = rospy.Rate(10)
        self.ego_status = EgoVehicleStatus()

        # 처음에 auto_mode , drive gear로 세팅
        self.send_gear_cmd(Gear.D.value)

        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_collision_data:
                print("[1] can't subscribe '/CollisionData' topic... \n    please check connection")
            if not self.is_ego:
                print("[2] can't subscribe '/Ego_topic' topic... \n    please check connection")


            if self.is_collision:
                # 충돌 발생시 기어
                self.send_gear_cmd(Gear.R.value)

                for _ in range(20):
                    self.send_ctrl_cmd(0.4, 10)
                    self.rate.sleep()

                self.send_gear_cmd(Gear.D.value)

            else:
                self.send_ctrl_cmd(0, 10)
                self.rate.sleep()

    # 충돌 메시지 콜백 함수
    def collision_callback(self, data):
        self.is_collision_data = True

        if(len(data.collision_object) > 0):
            self.is_collision = True
            print("collision : ", self.is_collision)
        else:
            self.is_collision = False

    # EGO 차량 상태 정보 콜백 함수
    def ego_callback(self, data):
        self.is_ego = True
        self.ego_status = data

    # 기어 변경 이벤트 메시지 세팅 함수
    def send_gear_cmd(self, gear_mode):
        # 기어 변경이 제대로 되기 위해서는 차량 속도가 약 0 이어야함
        while( abs(self.ego_status.velocity.x) > 0.1):
            self.send_ctrl_cmd(0,0)
            self.rate.sleep()
        
        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        gear_cmd_resp = self.event_cmd_srv(gear_cmd)
        rospy.loginfo(gear_cmd)

    # ctrl_cmd 메시지 세팅 함수
    def send_ctrl_cmd(self, steering ,velocity):
        cmd = CtrlCmd()
        if(velocity > 0):
            cmd.longlCmdType = 2
            cmd.velocity = velocity
            cmd.steering = steering
        else:
            cmd.longlCmdType = 1
            cmd.brake = 1
            cmd.steering = 0
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    try:
        s_d = s_drive()
    except rospy.ROSInterruptException:
        pass
