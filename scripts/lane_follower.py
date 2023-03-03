#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from std_msgs.msg import Float64

from util import purePursuit


if __name__ == '__main__':
    
    rp = rospkg.RosPack()
    
    currentPath = rp.get_path("beginner_tutorials")
    
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    rospy.init_node('lane_follower', anonymous=True)

    print("you need to set lfd, current value is 0  >>>>>>  ctrller = purePursuit(lfd=0)")
    ctrller = purePursuit(lfd=0)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        os.system('clear')
        print("This code is just an example. If you want higher performance, please proceed with direct tuning and algorithm advancement.")
        if ctrller.current_vel is not None and ctrller.lpath is not None:
            ctrller.calc_acc(20/3.6)
            
            ctrller.steering_angle()
            
            ctrller.pub_cmd()

            rate.sleep()
