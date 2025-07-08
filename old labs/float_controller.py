#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def messenger():
    channel = rospy.Publisher('float_chatter', Float32, queue_size=1)
    rospy.init_node("messenger", anonymous=True)

    while not rospy.is_shutdown():
	# this is the **python** float type
	user_input = float(input('Please enter a number > ')) 
        channel.publish(user_input)

if __name__ == '__main__':
    try:
	messenger()
    except rospy.ROSInterruptException:
        pass
