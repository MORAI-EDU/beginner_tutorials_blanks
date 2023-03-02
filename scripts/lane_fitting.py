#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from util import BEVTransform, CURVEFit, draw_lane_img

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None
        self.img_lane = None
        self.edges = None

        print("you need to find the right value : line 23 ~ 29")
        self.lower_wlane = np.array([0,0,0])
        self.upper_wlane = np.array([0,0,0])

        self.lower_ylane = np.array([0,0,0])
        self.upper_ylane = np.array([0,0,0])

        self.crop_pts = np.array([[[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]])

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

    def binarize(self, img):

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        img_wlane = cv2.inRange(img_hsv, self.lower_wlane, self.upper_wlane)
        img_ylane = cv2.inRange(img_hsv, self.lower_ylane, self.upper_ylane)

        self.img_lane = cv2.bitwise_or(img_wlane, img_ylane)

        return self.img_lane

    def mask_roi(self, img):

        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape)==3:

            # num of channel = 3

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255, 255, 255)

        else:
    
            # grayscale

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)

        mask = cv2.bitwise_and(mask, img)

        return mask


if __name__ == '__main__':
    
    rp = rospkg.RosPack()
    
    currentPath = rp.get_path("beginner_tutorials")
    
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)
    curve_learner = CURVEFit(order=3, lane_width=3.5 ,y_margin=1, x_range=30, min_pts=50)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if image_parser.img_bgr is not None:

            img_crop = image_parser.mask_roi(image_parser.img_bgr)

            img_warp = bev_op.warp_bev_img(img_crop)

            img_lane = image_parser.binarize(img_warp)

            img_f = bev_op.warp_inv_img(img_lane)

            lane_pts = bev_op.recon_lane_pts(img_f)

            x_pred, y_pred_l, y_pred_r = curve_learner.fit_curve(lane_pts)

            curve_learner.write_path_msg(x_pred, y_pred_l, y_pred_r)

            curve_learner.pub_path_msg()

            xyl, xyr = bev_op.project_lane2img(x_pred, y_pred_l, y_pred_r)

            img_lane_fit = draw_lane_img(img_lane, xyl[:, 0].astype(np.int32),
                                                xyl[:, 1].astype(np.int32),
                                                xyr[:, 0].astype(np.int32),
                                                xyr[:, 1].astype(np.int32))

            cv2.imshow("birdview", img_lane_fit)
            cv2.imshow("img_warp", img_warp)
            cv2.imshow("origin_img", image_parser.img_bgr)

            cv2.waitKey(1)

            rate.sleep()
