#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np
import os

from sensor_msgs.msg import CompressedImage


class Lane_binarize:
    def __init__(self):
        rospy.init_node('lane_binarize', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.is_image = False
        
        self.img_bgr = None

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_image:
                print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
            else:
                print(f"Caemra sensor was connected !")

            self.is_image = False
            rate.sleep()


    def callback(self, msg):
        self.is_image = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([0,0,0])
        upper_wlane = np.array([30,60,255])
        if np.sum(lower_wlane) == 0:
            print("you need to find the right value : line 39 ~ 40")
            exit()

        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)

        img_concat = np.concatenate([self.img_bgr, img_hsv, img_wlane], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 

if __name__ == '__main__':
    try:
        Lane_binarize = Lane_binarize()
    except rospy.ROSInterruptException:
        pass