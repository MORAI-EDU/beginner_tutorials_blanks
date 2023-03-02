#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:

    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

        print("you need to find the right value : line 28 ~ 29")
        lower_wlane = np.array([0,0,0])
        upper_wlane = np.array([0,0,0])

        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)

        img_concat = np.concatenate([self.img_bgr, img_hsv, img_wlane], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 

if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 
