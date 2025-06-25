#!/usr/bin/env python

''' Christian 6/20
remote_control.py: parses and outputs controller input 

SUBSCRIBES:
- `joy`: controller input of type `Joy` (great unambiguous naming!)

PUBLISHES:
- `robot_commands`: config input of type Int32MultiArray[5]
- `robot_twist`: movement input of type Twist 
===============================================================

The format of the config_command = i32[5] is as follows:
	- config_command[0] = enable/disable automatic stop on bumper event 
	- config_command[1] = enable/disable backwards movement when stopped by itself 
	- config_command[2] = enable/disable flashing LEDs and sound when backing up
	- config_command[3] = enable/disable e-brake
	- config_command[4] = smoothing mode (0=off, 1=eco, 2=sport)

All enable/disable config_commands follow the convention of 1=enable, 0=disable.
'''

import math
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray # only need 5

twist_publisher = rospy.Publisher("/robot_twist", Twist, queue_size=10)
config_publisher = rospy.Publisher("/robot_commands", Int32MultiArray, queue_size=10)
twist_command = Twist() # for linear/angular
config_command = Int32MultiArray() # for lights/sound/etc
config_command.data = [1, 1, 1, 0, 1]

smoothing_mode = 0 # indirect alias for config_command.data

def joystickCallback(data):
	global twist_publisher, config_publisher, twist_command, config_command
	global smoothing_mode 

	rt_val = data.axes[5]
	lt_val = data.axes[2]

	turn_val = data.axes[0] # left stick
	dpad_updown = data.axes[7]
	dpad_leftright = data.axes[6]

	a_val = data.buttons[0]
	b_val = data.buttons[1]
	x_val = data.buttons[2]
	y_val = data.buttons[3]
	rb_val = data.buttons[5]

	config_changed = False

	if y_val > 0: # toggle automatic stop on bumper event
		config_changed = True
		config_command.data[0] = 1 if config_command.data[0] == 0 else 0

	if x_val > 0: # toggle backwards movement when stopped by itself TODO what does this even mean
		config_changed = True
		config_command.data[1] = 1 if config_command.data[1] == 0 else 0

	if rb_val > 0: # toggle flashing and beeping when reversing
		config_changed = True
		config_command.data[2] = 1 if config_command.data[2] == 0 else 0

	if b_val > 0: # toggle e-brake 
		config_changed = True
		config_command.data[3] = 1 if config_command.data[3] == 0 else 0

	# configure smoothing mode
	if dpad_updown > 0: # sportmode
		print 'dpad up pressed; entering sport mode'
		config_changed = True
		smoothing_mode = 2
	elif dpad_leftright < 0: # ecomode
		print 'dpad right pressed; entering eco mode'
		config_changed = True
		smoothing_mode = 1
	elif dpad_updown < 0: # offmode
		print 'dpad down pressed; entering no-smoothing mode'
		config_changed = True
		smoothing_mode = 0
	config_command.data[4] = smoothing_mode

	if lt_val < 1.0: # slow down
		if lt_val <= 0:
			twist_command.linear.x = 0
	elif rt_val < 1.0 and a_val > 0: # move backwards
		rt_val = math.fabs(rt_val)
		if rt_val > .8:
			rt_val = .8
		twist_command.linear.x = -rt_val
	elif rt_val < 1.0: # move forwards
		rt_val = math.fabs(rt_val)
		if rt_val > .8:
			rt_val = .8
		twist_command.linear.x = rt_val
	elif turn_val != 0: # turn
		twist_command.angular.z = turn_val

	print data.buttons[0]
	print data.axes[0]
	
	twist_publisher.publish(twist_command)
	if config_changed:
		config_publisher.publish(config_command)

def cleanUp():
	global twist_publisher, twist_command
	twist_command.linear.x = 0.0
	twist_command.angular.z = 0.0
	twist_publisher.publish(twist_command)
	rospy.sleep(1)

def remoteController():
	global twist_publisher
	rospy.init_node("remoteControl", anonymous=True)
	rospy.Subscriber("joy", Joy, joystickCallback)
	rospy.on_shutdown(cleanUp)

	while twist_publisher.get_num_connections() == 0:
		pass

	rospy.spin()

if __name__ == '__main__':
	try:
		remoteController()
	except rospy.ROSInterruptException:
		pass
