#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math
from sklearn import linear_model
import random

# ignore warning
from warnings import simplefilter
from sklearn.exceptions import ConvergenceWarning
simplefilter("ignore", category=ConvergenceWarning)



class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.is_image = False
        self.img_bgr = None
        self.img_lane = None
        self.edges = None

        #####################################################################
        self.lower_wlane = np.array([0,0,0])
        self.upper_wlane = np.array([0,0,0])

        self.lower_ylane = np.array([0,0,0])
        self.upper_ylane = np.array([0,0,0])

        self.source_prop = np.float32([[[0,0],[0,0],[0,0],[0,0]]])
        
        if np.sum(self.lower_wlane) == 0 or np.sum(self.upper_wlane) == 0 or \
        np.sum(self.lower_ylane) == 0 or np.sum(self.upper_ylane) == 0 or \
        np.sum(self.source_prop) == 0:
            print("you need to find the right value : check lines at 33 ~ 39")
            exit()    
                                                  
        #####################################################################
    

    def callback(self, msg):
        self.is_image = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def binarize(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_wlane = cv2.inRange(img_hsv, self.lower_wlane, self.upper_wlane)
        img_ylane = cv2.inRange(img_hsv, self.lower_ylane, self.upper_ylane)
        self.img_lane = cv2.bitwise_or(img_wlane, img_ylane)

        return self.img_lane

    def warp_image(self, img, source_prop):
        
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



def rotationMtx(yaw, pitch, roll):
    
    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
    
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
 
    return R

def traslationMtx(x, y, z):
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M

def project2img_mtx(params_cam):
    
    '''
    project the lidar points to 2d plane
    \n xc, yc, zc : xyz components of lidar points w.r.t a camera coordinate
    \n params_cam : parameters from cameras 

    '''
    # focal lengths
    if params_cam["ENGINE"]=='UNITY':
        fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
        fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    else:
        fc_x = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
        fc_y = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    #the center of image
    cx = params_cam["WIDTH"]/2
    cy = params_cam["HEIGHT"]/2
    
    #transformation matrix from 3D to 2D
    R_f = np.array([[fc_x,  0,      cx],
                    [0,     fc_y,   cy]])

    return R_f



class BEVTransform:
    def __init__(self, params_cam, xb=10.0, zb=10.0):

        self.xb = xb
        self.zb = zb

        self.theta = np.deg2rad(params_cam["PITCH"])
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.x = params_cam["X"]
        
        if params_cam["ENGINE"]=="UNITY":
            self.alpha_r = np.deg2rad(params_cam["FOV"]/2)

            self.fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            self.alpha_c = np.arctan2(params_cam["WIDTH"]/2, self.fc_y)

            self.fc_x = self.fc_y

        else:
            self.alpha_c = np.deg2rad(params_cam["FOV"]/2)

            self.fc_x = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
            self.alpha_r = np.arctan2(params_cam["HEIGHT"]/2, self.fc_x)

            self.fc_y = self.fc_x
            
        self.h = params_cam["Z"] + 0.34

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT_b2g = np.matmul(np.matmul(traslationMtx(xb, 0, zb), rotationMtx(np.deg2rad(-90), 0, 0)),
                                rotationMtx(0, 0, np.deg2rad(180)))

        self.proj_mtx = project2img_mtx(params_cam)

        self._build_tf(params_cam)


    def calc_Xv_Yu(self, U, V):

        Xv = self.h*(np.tan(self.theta)*(1-2*(V-1)/(self.m-1))*np.tan(self.alpha_r)-1)/\
            (-np.tan(self.theta)+(1-2*(V-1)/(self.m-1))*np.tan(self.alpha_r))

        Yu = (1-2*(U-1)/(self.n-1))*Xv*np.tan(self.alpha_c)

        return Xv, Yu


    def _build_tf(self, params_cam):

        v = np.array([params_cam["HEIGHT"]*0.5, params_cam["HEIGHT"]]).astype(np.float32)
        u = np.array([0, params_cam["WIDTH"]]).astype(np.float32)

        U, V = np.meshgrid(u, v)

        Xv, Yu = self.calc_Xv_Yu(U, V)

        xyz_g = np.concatenate([Xv.reshape([1,-1]) + params_cam["X"],
                                Yu.reshape([1,-1]),
                                np.zeros_like(Yu.reshape([1,-1])),
                                np.ones_like(Yu.reshape([1,-1]))], axis=0)
        
        xyz_bird = np.matmul(np.linalg.inv(self.RT_b2g), xyz_g)

        xyi = self.project_pts2img(xyz_bird)

        src_pts = np.concatenate([U.reshape([-1, 1]), V.reshape([-1, 1])], axis=1).astype(np.float32)
        dst_pts = xyi.astype(np.float32)

        self.perspective_tf = cv2.getPerspectiveTransform(src_pts, dst_pts)

        self.perspective_inv_tf = cv2.getPerspectiveTransform(dst_pts, src_pts)


    def warp_bev_img(self, img):

        img_warp = cv2.warpPerspective(img, self.perspective_tf, (self.width, self.height), flags=cv2.INTER_LINEAR)
        
        return img_warp

    
    def warp_inv_img(self, img_warp):
    
        img_f = cv2.warpPerspective(img_warp, self.perspective_inv_tf, (self.width, self.height), flags=cv2.INTER_LINEAR)
        
        return img_f


    def recon_lane_pts(self, img):

        if cv2.countNonZero(img) != 0:
    
            UV_mark = cv2.findNonZero(img).reshape([-1,2])

            U, V = UV_mark[:, 0].reshape([-1,1]), UV_mark[:, 1].reshape([-1,1])
            
            Xv, Yu = self.calc_Xv_Yu(U, V)

            xyz_g = np.concatenate([Xv.reshape([1,-1]) + self.x,
                                Yu.reshape([1,-1]),
                                np.zeros_like(Yu.reshape([1,-1])),
                                np.ones_like(Yu.reshape([1,-1]))], axis=0)

            xyz_g = xyz_g[:, xyz_g[0,:]>=0]

        else:
            xyz_g = np.zeros((4, 10))

        return xyz_g


    def project_lane2img(self, x_pred, y_pred_l, y_pred_r):

        xyz_l_g = np.concatenate([x_pred.reshape([1,-1]),
                                  y_pred_l.reshape([1,-1]),
                                  np.zeros_like(y_pred_l.reshape([1,-1])),
                                  np.ones_like(y_pred_l.reshape([1,-1]))
                                  ], axis=0)

        xyz_r_g = np.concatenate([x_pred.reshape([1,-1]),
                                  y_pred_r.reshape([1,-1]),
                                  np.zeros_like(y_pred_r.reshape([1,-1])),
                                  np.ones_like(y_pred_r.reshape([1,-1]))
                                  ], axis=0)

        xyz_l_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_l_g)
        xyz_r_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_r_g)

        xyl = self.project_pts2img(xyz_l_b)
        xyr = self.project_pts2img(xyz_r_b)

        xyl = self.crop_pts(xyl)
        xyr = self.crop_pts(xyr)
        
        return xyl, xyr
        

    def project_pts2img(self, xyz_bird):

        xc, yc, zc = xyz_bird[0,:].reshape([1,-1]), xyz_bird[1,:].reshape([1,-1]), xyz_bird[2,:].reshape([1,-1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = xyi[0:2,:].T
        
        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi



class CURVEFit:
    
    def __init__(
        self,
        order=3,
        alpha=10,
        lane_width=13,
        y_margin=0.5,
        x_range=5,
        dx=0.5,
        min_pts=50,
        max_tri=5
    ):

        self.order = order
        self.lane_width = lane_width
        self.y_margin = y_margin
        self.x_range = x_range
        self.dx = dx
        self.min_pts = min_pts
        self.max_trials = max_tri

        self.lane_path = Path()

        self.ransac_left = linear_model.RANSACRegressor(base_estimator=linear_model.Lasso(alpha=alpha),
                                                        max_trials=self.max_trials,
                                                        loss='absolute_loss',
                                                        min_samples=self.min_pts,
                                                        residual_threshold=self.y_margin)

        self.ransac_right = linear_model.RANSACRegressor(base_estimator=linear_model.Lasso(alpha=alpha),
                                                        max_trials=5,
                                                        loss='absolute_loss',
                                                        min_samples=self.min_pts,
                                                        residual_threshold=self.y_margin)
        
        self._init_model()

        self.path_pub = rospy.Publisher('/lane_path', Path, queue_size=30)

    def _init_model(self):
        
        X = np.stack([np.arange(0, 2, 0.02)**i for i in reversed(range(1, self.order+1))]).T
        y_l = 0.5*self.lane_width*np.ones_like(np.arange(0, 2, 0.02))
        y_r = -0.5*self.lane_width*np.ones_like(np.arange(0, 2, 0.02))

        self.ransac_left.fit(X, y_l)
        self.ransac_right.fit(X, y_r)


    def preprocess_pts(self, lane_pts):

        idx_list = []

        for d in np.arange(0, self.x_range, self.dx):

            idx_full_list = np.where(np.logical_and(lane_pts[0, :]>=d, lane_pts[0, :]<d+self.dx))[0].tolist()
            
            idx_list += random.sample(idx_full_list, np.minimum(self.min_pts, len(idx_full_list)))

        lane_pts = lane_pts[:, idx_list]
        
        x_g = np.copy(lane_pts[0, :])
        y_g = np.copy(lane_pts[1, :])

        X_g = np.stack([x_g**i for i in reversed(range(1, self.order+1))]).T
        
        y_ransac_collect_r = self.ransac_right.predict(X_g)

        y_right = y_g[np.logical_and(y_g>=y_ransac_collect_r-self.y_margin, y_g<y_ransac_collect_r+self.y_margin)]
        x_right = x_g[np.logical_and(y_g>=y_ransac_collect_r-self.y_margin, y_g<y_ransac_collect_r+self.y_margin)]

        y_ransac_collect_l = self.ransac_left.predict(X_g)

        y_left = y_g[np.logical_and(y_g>=y_ransac_collect_l-self.y_margin, y_g<y_ransac_collect_l+self.y_margin)]
        x_left = x_g[np.logical_and(y_g>=y_ransac_collect_l-self.y_margin, y_g<y_ransac_collect_l+self.y_margin)]

        return x_left, y_left, x_right, y_right

    def fit_curve(self, lane_pts):

        x_left, y_left, x_right, y_right = self.preprocess_pts(lane_pts)
        
        if len(y_left)==0 or len(y_right)==0:

            self._init_model()

            x_left, y_left, x_right, y_right = self.preprocess_pts(lane_pts)

        X_left = np.stack([x_left**i for i in reversed(range(1, self.order+1))]).T
        X_right = np.stack([x_right**i for i in reversed(range(1, self.order+1))]).T

        if y_left.shape[0]>=self.ransac_left.min_samples:
            self.ransac_left.fit(X_left, y_left)
        
        if y_right.shape[0]>=self.ransac_right.min_samples:
            self.ransac_right.fit(X_right, y_right)
    
        #predict the curve
        x_pred = np.arange(0, self.x_range, self.dx).astype(np.float32)
        X_pred = np.stack([x_pred**i for i in reversed(range(1, self.order+1))]).T
        
        y_pred_l = self.ransac_left.predict(X_pred)
        y_pred_r = self.ransac_right.predict(X_pred)

        if y_left.shape[0]>=self.ransac_left.min_samples and y_right.shape[0]>=self.ransac_right.min_samples:

            self.update_lane_width(y_pred_l, y_pred_r)

        if y_left.shape[0]<self.ransac_left.min_samples:
            
            y_pred_l = y_pred_r + self.lane_width

        if y_right.shape[0]<self.ransac_right.min_samples:

            y_pred_r = y_pred_l - self.lane_width
        
        # overlap the lane

        if len(y_pred_l) == len(y_pred_r):

            if np.mean(y_pred_l + y_pred_r):

                if y_pred_r[x_pred==3.0]>0:
                    
                    y_pred_r = y_pred_l - self.lane_width

                elif y_pred_l[x_pred==3.0]<0:
                    
                    y_pred_l = y_pred_r + self.lane_width
            else:
                pass
        
        else:
            pass

        return x_pred, y_pred_l, y_pred_r

    def update_lane_width(self, y_pred_l, y_pred_r):
        temp = np.clip(np.max(y_pred_l-y_pred_r), 3.5, 5)

        self.lane_width = 13 if temp < 10 else temp

            
    
    def write_path_msg(self, x_pred, y_pred_l, y_pred_r, frame_id='/map'):

        self.lane_path = Path()

        self.lane_path.header.frame_id=frame_id

        for i in range(len(x_pred)) :
            tmp_pose=PoseStamped()
            tmp_pose.pose.position.x=x_pred[i]
            tmp_pose.pose.position.y=(0.5)*(y_pred_l[i] + y_pred_r[i])
            tmp_pose.pose.position.z=0
            tmp_pose.pose.orientation.x=0
            tmp_pose.pose.orientation.y=0
            tmp_pose.pose.orientation.z=0
            tmp_pose.pose.orientation.w=1
            self.lane_path.poses.append(tmp_pose)
    
    def pub_path_msg(self):

        self.path_pub.publish(self.lane_path)


def draw_lane_img(img, leftx, lefty, rightx, righty):
    '''
    place the lidar points into numpy arrays in order to make intensity map
    \n img : source image
    \n leftx, lefty, rightx, righty : curve fitting result 
    '''
    point_np = cv2.cvtColor(np.copy(img), cv2.COLOR_GRAY2BGR)

    #Left Lane
    for ctr in zip(leftx, lefty):
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)

    #Right Lane
    for ctr in zip(rightx, righty):
        point_np = cv2.circle(point_np, ctr, 2, (0,0,255),-1)

    return point_np




if __name__ == '__main__':
    
    rp = rospkg.RosPack()
    
    currentPath = rp.get_path("beginner_tutorials")
    
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)
    curve_learner = CURVEFit(order=3, lane_width=13 ,y_margin=1, x_range=30, min_pts=50)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        os.system('clear')

        if image_parser.img_bgr is not None:
            
            img_warp = image_parser.warp_image(image_parser.img_bgr, image_parser.source_prop)

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
            print(f"Caemra sensor was connected !")

        else:
            print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
                
        image_parser.img_bgr = None
        rate.sleep()
