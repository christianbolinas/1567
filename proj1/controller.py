#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
reset_pub = rospy.Publisher("/mobile_base/commands/reset_odometry", Empty, queue_size=10)

odometry = Odometry()

def commandParse(commands):
	for i in commands:
		speed = float(commands[0])
		distance = float(commands[1])
		roation = bool(commands[2])

		pub.publish(Empty())

		print("speed: {}", speed)

def getUserInput():
	commands = []

	while True:
		command_input = input('Enter command (s to submit): ').split()

		if command_input[0] == 's':
			break
		else:
			commands.append()

	commandParse(commands)

def odomCallback(data):
	odometry = data

def controller():
	rospy.init_node("controller", anonymous=True)
	rospy.Subscriber("/odom", Odometry, odomCallback)
	#rospy.on_shutdown(cleanUp)

	#rospy.spin()

	#while velocity_pub.get_num_connections() == 0 or reset_pub.get_num_connections() == 0:
	#	pass

	while True:
		getUserInput()

if __name__ == '__main__':
	controller()