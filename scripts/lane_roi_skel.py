#!/usr/bin/env python3
 
import rospy, os
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


class Lane_roi:
    def __init__(self):
        rospy.init_node('lane_roi', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.is_image = False
        self.crop_pts = np.array(
            [[
                [0,0],
                [0,0],
                [0,0],
                [0,0]
            ]]
        )
        if np.sum(self.crop_pts) == 0:
            print("you need to change values at line 19~22 : masking points")
            exit()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_image:
                cv2.destroyAllWindows()
                print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
            else:
                print(f"Caemra sensor was connected !")

            rate.sleep()


    def callback(self, msg):
        self.is_image = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        self.mask = self.mask_roi(img_bgr)

        if len(self.mask.shape)==3:
            img_concat = np.concatenate([img_bgr, self.mask], axis=1)

        else:
            img_concat = np.concatenate([img_bgr, cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)], axis=1)

        cv2.imshow("lane_roi", img_concat)
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
    try:
        Lane_roi = Lane_roi()
    except rospy.ROSInterruptException:
        pass