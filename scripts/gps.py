#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import os
from pyproj import Proj
from std_msgs.msg import Float32MultiArray

from morai_msgs.msg import GPSMessage, EgoVehicleStatus

class LL2UTMConverter:
    def __init__(self, zone=52) :

        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)

        self.x, self.y = None, None
        
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)


    def navsat_callback(self, gps_msg):

        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude

        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.convertLL2UTM()

        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 0.),
                        tf.transformations.quaternion_from_euler(0,0,0.),
                        rospy.Time.now(),
                        "map",
                        "base_link")

        utm_msg = Float32MultiArray()

        utm_msg.data = [self.x, self.y]
        os.system('clear')
        if self.lat and self.lon:
            print('위도 : ', self.lat)
            print('경도 : ', self.lon)
        else:
            print('위도 : ', '?????')
            print('경도 : ', '?????')
            print("GPS BLACK OUT!!")


    def convertLL2UTM(self):

        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0] - self.e_o
        self.y = xy_zone[1] - self.n_o


if __name__ == '__main__':

    rospy.init_node('gps_parser', anonymous=True)

    gps_parser = LL2UTMConverter()

    rospy.spin()
        
