import numpy as np
import cv2
import math
import time
from sklearn import linear_model
import random

from std_msgs.msg import Float64
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

import rospy


from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point

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
        lane_width=4,
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

        self.lane_width = np.clip(np.max(y_pred_l-y_pred_r), 3.5, 5)
    
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


class purePursuit :
    def __init__(
        self,
        lfd,
        vehicle_length=2,
        min_lfd=2,
        max_lfd=50
        ):

        self.is_look_forward_point = False
        self.vehicle_length = vehicle_length

        self.lfd = lfd
        self.min_lfd = min_lfd
        self.max_lfd = max_lfd

        self.status_sub = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB)
        self.lpath_sub = rospy.Subscriber('/lane_path', Path, self.lane_path_callback)
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        
        self.lpath = None
        self.ctrl_msg = CtrlCmd()
        self.current_vel = None


    def statusCB(self,data):

        self.current_vel = data.velocity.x


    def lane_path_callback(self, msg):
        
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
            print("you need to change value at line 482 : purepursuit_steering")
            steering_deg= 0

            self.ctrl_msg.steering = steering_deg
        else : 
            self.ctrl_msg.steering = 0.0
            print("no found forward point")

    def calc_acc(self, target_vel):

        err = target_vel - self.current_vel

        control_input = 1*err

        if control_input > 0 :
            self.ctrl_msg.accel= control_input
            self.ctrl_msg.brake= 0
        else :
            self.ctrl_msg.accel= 0
            self.ctrl_msg.brake= -control_input


    def pub_cmd(self):
        self.cmd_pub.publish(self.ctrl_msg)

