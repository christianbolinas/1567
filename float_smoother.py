#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import math

value = 0.0
target = 0.0

def callback(data):
    global target
    target = data.data

def listener():
    global value
    global target

    rospy.init_node("float_smoother", anonymous=True)
    rospy.Subscriber("float_chatter", Float32, callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        print value

	if value > target:
		value = value - 0.01
		if value < target:
			value = target
	elif value < target:
		value = value + 0.01
		if value > target:
			value = target
	rate.sleep()

if __name__ == '__main__':
    listener()
