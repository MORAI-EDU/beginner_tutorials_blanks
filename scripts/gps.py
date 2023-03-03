#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os

from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage



class GPS_to_UTM:
    def __init__(self):
        rospy.init_node('GPS_to_UTM', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False

        rospy.spin()


    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        longitude = gps_msg.longitude
        latitude = gps_msg.latitude
        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        map_x = utm_x - gps_msg.eastOffset
        map_y = utm_y - gps_msg.northOffset
        
        os.system('clear')
        print("-------------------------------------")
        print(f"longitude : {gps_msg.longitude}")
        print(f"latitude : {gps_msg.latitude}")
        print()
        print(f"utm_x : {utm_x}")
        print(f"utm_y : {utm_y}")
        print(f"simulator map_x : {map_x}")
        print(f"simulator map_y : {map_y}")
        print("-------------------------------------")


if __name__ == '__main__':
    try:
        GPS_to_UTM = GPS_to_UTM()
    except rospy.ROSInterruptException:
        pass
