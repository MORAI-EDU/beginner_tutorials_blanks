#!/usr/bin/env python

import rospy
from morai_msgs.msg import <<here!>>

class s_drive():
    def __init__(self):
        rospy.init_node('s_drive', anonymous=True)
        cmd_pub = rospy.Publisher(<<here!>>, <<here!>>, queue_size=1)
        rate = rospy.Rate(30)
        cmd = <<here!>>
        cmd.longlCmdType = <<here!>>
        cmd.velocity = <<here!>>
        steering_cmd = [ <<here!>>, <<here!>>]
        cmd_cnts = <<here!>>



if __name__ == '__main__':
    try:
        s_d = s_drive()
    except rospy.ROSInterruptException:
        pass
