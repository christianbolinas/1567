#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
command = Twist()

def joystickCallback(data):
	global pub, command

	rt_val = data.axes[5]
	lt_val = data.axes[2]
	a_val = data.buttons[0]
	b_val = data.buttons[1]
	turn_val = data.axes[0]

	if b_val > 0: #emergency stop
		rospy.signal_shutdown("Emergency Stop!!!")

	if(lt_val < 1.0): #slow down
		if( lt_val <= 0):
			command.linear.x = 0
	elif rt_val < 1.0 and a_val > 0: #move backwards
		rt_val = math.fabs(rt_val)
		if rt_val > .8:
			rt_val = .8
		command.linear.x = -rt_val
	elif rt_val < 1.0: #move forwards
		rt_val = math.fabs(rt_val)
		if( rt_val > .8):
			rt_val = .8
		command.linear.x = rt_val
	elif(turn_val != 0): #turn
		command.angular.z = turn_val


	print data.buttons[0];
	print data.axes[0];
	pub.publish(command)

def cleanUp():
	global pub, command
	command.linear.x = 0.0
	command.angular.z = 0.0
	pub.publish(command)
	rospy.sleep(1)

def remoteController():
	rospy.init_node("remoteControl", anonymous=True)
	rospy.Subscriber("joy", Joy, joystickCallback)
	rospy.on_shutdown(cleanUp)

	while pub.get_num_connections() == 0:
		pass

	#our code fragment here 
	#while not rospy.is_shutdown:
	# 	keep publishing twist

	rospy.spin()

if __name__ == '__main__':
	remoteController()
