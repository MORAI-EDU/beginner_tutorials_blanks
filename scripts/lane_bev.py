#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np
import os

from sensor_msgs.msg import CompressedImage




def warp_image(img, source_prop):
    
    image_size = (img.shape[1], img.shape[0])

    x = img.shape[1]
    y = img.shape[0]
    
    destination_points = np.float32([
    [0, y],
    [0, 0],
    [x, 0],
    [x, y]
    ])

    source_points = source_prop * np.float32([[x, y]]* 4)
    
    perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)
    
    warped_img = cv2.warpPerspective(img, perspective_transform, image_size, flags=cv2.INTER_LINEAR)
    
    return warped_img


class Lane_birdview:
    def __init__(self):
        rospy.init_node('lane_birdview', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)        
        self.is_image = False
        
        self.img_bgr = None
        self.source_prop = np.float32([[0.01, 0.80],
                                       [0.5 - 0.14, 0.52],
                                       [0.5 + 0.14, 0.52],
                                       [1 - 0.01, 0.80]
                                       ])
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_image:
                print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
            else:
                print(f"""
                    Please change the camera parameters as follows:\n
                    FOV    : 90         roll  : 0.0        x : 1.8        
                    WIDTH  : 640        pitch : 7.0        y :  0        
                    HEIGHT : 480        yaw   : 0.0        z :  2
                    """)

            self.is_image = False
            rate.sleep()


    def callback(self, msg):
        self.is_image = True

        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img_warp = warp_image(self.img_bgr, self.source_prop)

        img_concat = np.concatenate([self.img_bgr, img_warp], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 


if __name__ == '__main__':
    try:
        Lane_birdview = Lane_birdview()
    except rospy.ROSInterruptException:
        pass