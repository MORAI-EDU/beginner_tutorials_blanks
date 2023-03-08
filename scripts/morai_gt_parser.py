#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from math import pi
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage,EgoVehicleStatus

class morai_gt_parser:
    def __init__(self):
        rospy.init_node('morai_gt_parser', anonymous=True)

        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)

        self.br = tf.TransformBroadcaster()

        self.is_status=False

        self.x, self.y = None, None

        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link1'

        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():

            if self.is_status == True:

                self.odom_pub.publish(self.odom_msg)

                rate.sleep()

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg

        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, self.status_msg.heading/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")

        self.x=self.status_msg.position.x
        self.y=self.status_msg.position.y

        q_x,q_y,q_z,q_w = quaternion_from_euler(0, 0, self.status_msg.heading/180*pi)

        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0
        self.odom_msg.pose.pose.orientation.x = q_x
        self.odom_msg.pose.pose.orientation.y = q_y
        self.odom_msg.pose.pose.orientation.z = q_z
        self.odom_msg.pose.pose.orientation.w = q_w
#        print("self.odom_msg : ",self.odom_msg)


if __name__ == '__main__':
    try:
        morai_gt_parser = morai_gt_parser()
    except rospy.ROSInterruptException:
        pass
