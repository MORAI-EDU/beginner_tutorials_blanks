#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import os


class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.07
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel
        p_control = self.p_gain * error
        if error <= 5:
            self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime
        output = p_control + self.i_control + d_control
        self.prev_error = error
        return output


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("Ego_topic", EgoVehicleStatus, self.status_callback)


        
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd_0',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=1

        self.is_path=False
        self.is_odom=False
        self.is_current_vel =False
        self.is_status = False
        self.target_vel = 40
        self.current_vel = 0.0
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length = None
        self.lfd = None
        if self.vehicle_length is None or self.lfd is None:
            print("you need to change values at line 57~58 ,  self.vegicle_length , lfd")
            exit()

        self.pid_controller = pidControl()

        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():
            if self.is_path ==True and self.is_odom==True and self.is_status == True :
                
                vehicle_position=self.current_postion
                self.is_look_forward_point= False

                translation=[vehicle_position.x, vehicle_position.y]

                t=np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])

                det_t=np.array([
                       [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                       [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                       [0      ,0      ,1                                               ]])

                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position

                    global_path_point=[path_point.x,path_point.y,1]
                    local_path_point=det_t.dot(global_path_point)           
                    if local_path_point[0]>0 :
                        dis=sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                        if dis>= self.lfd :
                            self.forward_point=path_point
                            self.is_look_forward_point=True
                            break
                
                theta=atan2(local_path_point[1],local_path_point[0])

                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)
                    output = self.pid_controller.pid(self.target_vel, self.current_vel * 3.6)

                    if output > 0:
                        self.ctrl_cmd_msg.accel = output
                        self.ctrl_cmd_msg.brake = 0
                    else:
                        self.ctrl_cmd_msg.accel = 0
                        self.ctrl_cmd_msg.brake = - output
                    
                    os.system('clear')
                    print("-------------------------------------")
                    print(" Accel (%) = ", 100 if self.ctrl_cmd_msg.accel * 100 >= 100 else self.ctrl_cmd_msg.accel * 100)
                    print(" Brake (%) = ", 100 if self.ctrl_cmd_msg.brake * 100 >= 100 else self.ctrl_cmd_msg.brake * 100)
                    print("-------------------------------------")

                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0
                    self.ctrl_cmd_msg.velocity=0.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            
            else:
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe '/local_path' topic...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")
                if not self.is_status:
                    print("[3] can't subscribe '/Ego_topic' topic...")
            
            self.is_path = self.is_odom = self.is_status = False
            rate.sleep()


    def status_callback(self, msg):
        self.is_status = True
        self.current_vel = msg.velocity.x

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
