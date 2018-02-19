#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry

def callbackOdometry(msg):
    print msg.pose.pose

if __name__ == "__main__":
    rospy.init_node('oodometry', anonymous=True)
    rospy.Subscriber('odom', Odometry, callbackOdometry)
    rospy.spin()
