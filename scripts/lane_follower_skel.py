#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json
import math

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from std_msgs.msg import Float64

from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
from nav_msgs.msg import Path

class PurePursuit :
    def __init__(self):
        rospy.init_node('lane_follower', anonymous=True)
        self.status_sub = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.lpath_sub = rospy.Subscriber('/lane_path', Path, self.lane_path_callback)
        self.cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)
        
        self.is_status = False
        self.is_lpath = False
        
        self.is_look_forward_point = False
        self.vehicle_length = 2
        self.lfd = 20
        self.min_lfd = 2
        self.max_lfd = 50

        self.lpath = None
        self.ctrl_msg = CtrlCmd()
        self.current_vel = 0

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            os.system('clear')
            if self.lpath:            
                self.calc_acc(20 / 3.6)    
                self.steering_angle()
                self.cmd_pub.publish(self.ctrl_msg)
            
            if self.is_status and self.is_lpath:
                print(f'''
                    lane_follower is processing...
                -------------------------------------
                    accel     : {self.ctrl_msg.accel}
                    brake     : {self.ctrl_msg.brake}
                   steering   : {self.ctrl_msg.steering}
                   velocity   : {self.ctrl_msg.velocity}
                 acceleration : {self.ctrl_msg.acceleration}
                ''')
                print('if nothing happens... please check [F4] Cmd Control Network')
            else:
                if not self.is_status:
                    print("[1] can't subscribe '/Ego_topic' topic... \n    please check connection")
                if not self.is_lpath:
                    print("[2] can't subscribe '/lane_path' topic... \n    need lane_fitting.py")

            self.is_status = False
            self.is_lpath = False
            rate.sleep()


    def status_callback(self,data):
        self.is_status = True
        self.current_vel = data.velocity.x

    def lane_path_callback(self, msg):
        self.is_lpath = True
        self.lpath = msg

    def steering_angle(self):

        self.is_look_forward_point= False

        for i in self.lpath.poses:

            path_point=i.pose.position
            
            if path_point.x>0 :

                dis_i = np.sqrt(np.square(path_point.x) + np.square(path_point.y))
                
                if dis_i>= self.lfd :

                    self.is_look_forward_point=True
                    
                    break
        
        theta=math.atan2(path_point.y, path_point.x)

        if self.is_look_forward_point :
            steering_deg = None
            if steering_deg is None:
                print("you need to change pure_pursuit line 98 : calcu_steering")
                exit()

            self.ctrl_msg.steering = steering_deg
        else : 
            self.ctrl_msg.steering = 0.0
            print("no found forward point")

    def calc_acc(self, target_vel):

        err = target_vel - self.current_vel

        control_input = 1 * err

        if control_input > 0 :
            self.ctrl_msg.accel= control_input
            self.ctrl_msg.brake= 0
        else :
            self.ctrl_msg.accel= 0
            self.ctrl_msg.brake= -control_input


        

if __name__ == '__main__':
    
    rp = rospkg.RosPack()
    currentPath = rp.get_path("beginner_tutorials")
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    try:
        PurePursuit = PurePursuit()
    except rospy.ROSInterruptException:
        pass