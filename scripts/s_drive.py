#!/usr/bin/env python

import rospy
from morai_msgs.msg import 

class s_drive():
    def __init__(self):
        rospy.init_node('s_drive', anonymous=True)
        cmd_pub = rospy.Publisher(, , queue_size=1)
        rate = rospy.Rate(30)
        cmd = 
        cmd.longlCmdType = 
        cmd.velocity = 
        steering_cmd = [ , ]
        cmd_cnts = 

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
