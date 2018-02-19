#!/usr/bin/python

import rospy

rospy.init_node("hello")

while not rospy.is_shutdown():
	print "Hello, world"
