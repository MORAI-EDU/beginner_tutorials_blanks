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

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1) 


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 