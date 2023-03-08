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

        self.crop_pts = np.array(
            [[
                [0,0],
                [0,0],
                [0,0],
                [0,0]
            ]]
        )

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.mask = self.mask_roi(img_bgr)

        if len(self.mask.shape)==3:
            img_concat = np.concatenate([img_bgr, self.mask], axis=1)

        else:
            img_concat = np.concatenate([img_bgr, cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)

    def mask_roi(self, img):

        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape)==3:

            # image shape : [h, w, 3]

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255, 255, 255)

        else:

            # binarized image or grayscale image : [h, w]

            mask = np.zeros((h, w), dtype=np.uint8)

            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)

        mask = cv2.bitwise_and(mask, img)

        return mask


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 
