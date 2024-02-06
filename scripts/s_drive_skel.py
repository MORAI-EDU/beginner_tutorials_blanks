#!/usr/bin/env python3

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
        cmd_cnts = 50

        while not rospy.is_shutdown():
            for i in range(2):
                cmd.steering = steering_cmd[i]
                rospy.loginfo(cmd)
                for _ in range(cmd_cnts):
                    cmd_pub.publish(cmd)
                    rate.sleep()

if __name__ == '__main__':
    try:
        s_d = s_drive()
    except rospy.ROSInterruptException:
        pass