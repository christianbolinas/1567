#!/usr/bin/env python

''' Christian 6/20
float_smoother.py: receives commands and controlls robot

SUBSCRIBES:
- `robot_twist`: receives commands of type `Twist`
- `robot_commands`: receives commands of type i32[5]

PUBLISHES:
- `/mobile_base/commands/velocity`: movement commands of type Twist (after smoothing)
- `/mobile_base/commands/sound`: sound commands of type Sound
- `/mobile_base/commands/{led1, led2}`: LED commands of type Led
=====================================================================================

- flashes LED #1, makes sound when robot moves backwards
- publishes `Twist` command every 10 ms
- supports e-brake (pressing `B` button makes robot stop immediately)
- has three types of smoothing mode, displayed by LED #2:
	- eco mode: accelerates and decelerates less quickly
	- sport mode: accelerates and decelerates more quickly
	- off mode: no smoothing
'''

import math
import rospy
from std_msgs.msg import Float32, Int32MultiArray
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led, Sound

# INIT SUBSCRIBERS
rospy.init_node('smoother', anonymous=True)
rospy.Subscriber('robot_twist', Twist, twist_callback) # TODO make twist_callback
rospy.Subscriber('robot_commands', Int32MultiArray, command_callback) # TODO make command_callback

# INIT PUBLISHERS
velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
sound_pub = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=1)
led1_pub = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=1)
led2_pub = rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=1)

# STATE
config_command = Int32MultiArray()
config_command.data = [0, 0, 0, 0, 0]
velocity_command = Twist()
velocity_input = Twist()

# CONFIG: UNITS OF MEASURE master rate is every 10ms (100hz)
DURATION_S = 0.01 # seconds: 0.01s == 10ms

# TODO: sets velocity_input equal to data on `robot_twist` update
def twist_callback(data):
	pass

# sets config_command equal to data on `robot_commands` and prints new state
def command_callback(data): 
	global config_command
	config_command = data

	bumper_on = 'on' if config_command.data[0] == 1 else 'off'
	backward_only = 'on' if config_command.data[1] == 1 else 'off'
	led_sound = 'on' if config_command.data[2] == 1 else 'off'
	e_brake = 'Engaged' if config_command.data[3] == 1 else 'Disengaged'

	smoothing_modes = ['None', 'Eco', 'Sport'] 
	smoothing_mode = smoothing_modes[config_command.data[4]]
	# CHRISTIAN: this is like seven stack allocations just for those two lines...
	
	print 'Bumper:\t\t\t{}'.format(bumper_on) # not buffering the IO lol
	print 'Backward Only:\t\t\t{}'.format(backward_only)
	print 'LED and Sound:\t\t\t{}'.format(led_sound)
	print 'Emergency Brake:\t\t\t{}'.format(e_brake)
	print 'Smoothing Mode:\t\t\t{}'.format(smoothing_mode)
	

# CALLBACK: smooths velocity, then publishes it and state value to kobuki. happens every 10ms
def smooth_and_publish(data):
	global velocity_pub, velocity_command, config_command
	# TODO: smooth velocity input and set velocity command to it
	velocity_pub.publish(velocity_command)

	# TODO: configure kobuki based on config_command
	backing_up = velocity_command.linear.x < 0
	if backing_up:
		pass
	# etc

# takes published values $(SMOOTHING_RATE) closer to input values, publishes every 10ms (0.01s)
if __name__ == '__main__':
	# calls smooth_and_publish every 0.01s
	publish_thread = rospy.Timer(rospy.Duration(DURATION_S), smooth_and_publish)

''' OLD float_smoother, from Ethan's lab 1.
value = 0.0
target = 0.0

ECO_DELTA = 0.01
SPORT_DELTA = 0.005

def callback(data):
    global target
    target = data.data

def listener():
    global value, target

    rospy.init_node("smoother", anonymous=True)
	rospy.init_node("robot_commands", Int32MultiArray, config
    rospy.Subscriber("robot_twist", Twist, callback)

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
'''
