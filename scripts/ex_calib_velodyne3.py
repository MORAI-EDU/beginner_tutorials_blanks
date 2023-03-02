#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import PointCloud2, CompressedImage, Image
from cv_bridge import CvBridgeError,CvBridge
import sensor_msgs.point_cloud2 as pc2
from numpy.linalg import inv


params_lidar = {
    "X": 0.0, # meter
    "Y": 0.0,
    "Z": 2.2 + 0.03,
    "YAW": 0.0, # deg
    "PITCH": 0.0,
    "ROLL": 0.0
}

params_cam = {
    # "WIDTH": 1920, # image width
    # "HEIGHT": 1080, # image height
    # "FOV": 38, # Field of view
    "WIDTH": 640, # image width
    "HEIGHT": 480, # image height
    "FOV": 60, # Field of view
    "X": 1.5 + 0.1, # meter
    "Y": 0,
    "Z": 1.3,
    "YAW": 0.0, # deg
    "PITCH": 0.0,
    "ROLL": 0.0
}

def getRotMat(RPY):
    cosR = math.cos(RPY[0])
    cosP = math.cos(RPY[1])
    cosY = math.cos(RPY[2])
    sinR = math.sin(RPY[0])
    sinP = math.sin(RPY[1])
    sinY = math.sin(RPY[2])
    
    rotRoll = np.array([1,0,0, 0,cosR,-sinR, 0,sinR,cosR]).reshape(3,3)
    rotPitch = np.array([cosP,0,sinP, 0,1,0, -sinP,0,cosP]).reshape(3,3)
    rotYaw = np.array([cosY,-sinY,0, sinY,cosY,0, 0,0,1]).reshape(3,3)
    
    rotMat = rotYaw.dot(rotPitch.dot(rotRoll))
    
    return rotMat

def transformLidarToCamNew(params_lidar, params_cam):
    #With Respect to Vehicle ISO Coordinate
    #camPosition = np.array([1.4, 0, 1.2])
    camRPY = np.array([-90*math.pi/180,0,(-90+8.5)*math.pi/180])
    #lidarPosition = np.array([1.4, 0, 1.3])
    lidarRPY = np.array([0,0,0])

    lidarPosition = [params_lidar.get(i) for i in (["X","Y","Z"])]
    camPosition = [params_cam.get(i) for i in (["X","Y","Z"])]

    lidarPositionOffset = np.array([0.0,0,0.02081])
    lidarPosition = lidarPosition + lidarPositionOffset

    camPositionOffset = np.array([0.1085, 0, 0])
    camPosition = camPosition + camPositionOffset

    camRot = getRotMat(camRPY)
    camTransl = np.array([camPosition])
    Tr_cam_to_vehicle = np.concatenate((camRot,camTransl.T),axis = 1)
    Tr_cam_to_vehicle = np.insert(Tr_cam_to_vehicle, 3, values=[0,0,0,1],axis = 0)

    lidarRot = getRotMat(lidarRPY)
    lidarTransl = np.array([lidarPosition]) 
    Tr_lidar_to_vehicle = np.concatenate((lidarRot,lidarTransl.T),axis = 1)
    Tr_lidar_to_vehicle = np.insert(Tr_lidar_to_vehicle, 3, values=[0,0,0,1],axis = 0)

    invTr = inv(Tr_cam_to_vehicle)
    Tr_lidar_to_cam = invTr.dot(Tr_lidar_to_vehicle).round(6)
    return Tr_lidar_to_cam

def TranslationMatrix(x, y, z):
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M

def RotationMatrix(yaw, pitch, roll):
     
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

##########################################################################################

def transformMTX_lidar2cam(params_lidar, params_cam):
    
    #Relative position of lidar w.r.t cam
    lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
    cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

    x_rel = cam_pos[0] - lidar_pos[0]
    y_rel = cam_pos[1] - lidar_pos[1]
    z_rel = cam_pos[2] - lidar_pos[2]

    R_T = np.matmul(TranslationMatrix(x_rel,y_rel,z_rel),RotationMatrix(-90*math.pi/180,0,-90*math.pi/180))
    R_T = np.matmul(R_T, RotationMatrix(0, 0., 0.))    

    #R_T = np.matmul(TranslationMatrix(x_rel,y_rel,z_rel),RotationMatrix(np.deg2rad(-90.),0.,0.))
    #R_T = np.matmul(R_T, RotationMatrix(0, 0., np.deg2rad(-90.)))
    
    #rotate and translate the coordinate of a lidar
    R_T = np.linalg.inv(R_T)


    return R_T

def project2img_mtx(params_cam):

    # focal lengths
    fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    #the center of image
    cx = params_cam["WIDTH"]/2
    cy = params_cam["HEIGHT"]/2
    
    #transformation matrix from 3D to 2D
    R_f = np.array([[320,  0,      320],
                    [0,     320,   240]])

    return R_f

def draw_pts_img(img, xi, yi):
    point_np = img

    #Left Lane
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 1, (255, 0, 255),-1)
        # point_np = cv2.circle(point_np, ctr, 20, (0,255,0),1)

    return point_np

class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):

        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT = transformLidarToCamNew(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p):
        
        xyz_c = np.matmul(np.concatenate([xyz_p, np.ones((xyz_p.shape[0], 1))], axis=1), self.RT.T)

        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):

        xyz_c = xyz_c.T

        xc, yc, zc = xyz_c[0,:].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]), xyz_c[2,:].reshape([1,-1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = xyi[0:2,:].T

        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass
        
        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi

class sensorcalib:
    def __init__(self):

        # rospy.init_node('ex_calib', anonymous=True)

        self.scan_sub = rospy.Subscriber("/velodyne_points",PointCloud2, self.scan_callback)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.pc_np = None
        self.img = None

        # self.img_test = False
        # self.lidar_test = False

        # time.sleep(1)

        # rate = rospy.Rate(10)

        # while not rospy.is_shutdown():

        #     if self.img_test != False and self.lidar_test != False:
            
        #         # xyz_p = self.pointcloud_to_xyz(np.where(self.pointcloud_to_xyz[:,0]>=0))

        #         # print(xyz_p)

        #         # intens_p = self.intens.reshape([-1,1])
        #         # intens_p = intens_p[np.where(self.pointcloud_to_xyz[:,0]>=0)]

        #         xyz_c = self.l2c_trans.transform_lidar2cam(self.pc_np)

        #         xy_i = self.l2c_trans.project_pts2img(xyz_c,crop=True)

        #         img_lc2 = draw_pts_img(self.img,xy_i[:,0].astype(np.int32),xy_i[:,1].astype(np.int32))

        #         cv2.imshow("Lidar2Cam",img_lc2)
        #         cv2.waitKey(1)

        #     rate.sleep()

    def img_callback(self, msg):

        # np_arr = np.fromstring(msg.data, np.uint8)
        # # img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.img_test = True

    def scan_callback(self, msg):

        self.pc_np = self.pointcloud_to_xyz(msg)

        self.lidar_test = True

    def pointcloud_to_xyz(self, cloud_msg):

        point_list = []

        for point in pc2.read_points(cloud_msg, skip_nans=True):

            # print(point)

            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)

            if point[0] > 0 and dist < 20:
                # point_list.append((point[0],point[1],point[2],point[3]))
                point_list.append((point[0],point[1],point[2]))

        point_np = np.array(point_list, np.float32)

        return point_np

if __name__ == '__main__':
    
    rospy.init_node('ex_calib', anonymous=True)

    ex_calib_transform = sensorcalib()

    time.sleep(1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        xyz_p = ex_calib_transform.pc_np
        
        # print(xyz_p)

        # intens_p = ex_calib_transform.intens.reshape([-1,1])
        # intens_p = intens_p[np.where(ex_calib_transform.pointcloud_to_xyz[:,0]>=0)]

        xyz_c = ex_calib_transform.l2c_trans.transform_lidar2cam(xyz_p)

        xy_i = ex_calib_transform.l2c_trans.project_pts2img(xyz_c,crop=True)

        img_lc2 = draw_pts_img(ex_calib_transform.img,xy_i[:,0].astype(np.int32),xy_i[:,1].astype(np.int32))

        cv2.imshow("Lidar2Cam",img_lc2)
        cv2.waitKey(1)