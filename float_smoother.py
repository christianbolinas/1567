#!/usr/bin/env python

''' Christian 6/20, 6/24
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
from kobuki_msgs.msg import Led, Sound, BumperEvent

velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
sound_pub = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=1)
led1_pub = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=1)
led2_pub = rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=1)

def twist_callback(data):
	global velocity_input
	velocity_input = data

# sets config_command equal to data on `robot_commands` and prints new state
SMOOTHING_MODES = ['None', 'Eco', 'Sport'] 

config_command = Int32MultiArray()
config_command.data = [1, 1, 1, 0, 1]

def command_callback(data): 
	global config_command, SMOOTHING_MODES, led1_pub
	# save input state
	config_command = data

	# deserialize controller config input
	bumper_on = 'on' if config_command.data[0] == 1 else 'off'
	backward_only = 'on' if config_command.data[1] == 1 else 'off'
	led_sound = 'on' if config_command.data[2] == 1 else 'off'
	e_brake = 'Engaged' if config_command.data[3] == 1 else 'Disengaged'
	smoothing_mode = SMOOTHING_MODES[config_command.data[4]]

	# publish to Led1 to indicate smoothing mode
	green, black, red = 1, 0, 3

	curr_led_color = black
	if smoothing_mode == 'Eco':
		curr_led_color = green
	elif smoothing_mode == 'Sport':
		curr_led_color = red
	# print 'publishing {} to led1_pub'.format(curr_led_color)
	led1_pub.publish(curr_led_color)

	# print new state. not buffering IO lol
	print 'Bumper:\t\t\t\t{}'.format(bumper_on) 
	print 'Backward Only:\t\t\t{}'.format(backward_only)
	print 'LED and Sound:\t\t\t{}'.format(led_sound)
	print 'Emergency Brake:\t\t{}'.format(e_brake)
	print 'Smoothing Mode:\t\t\t{}\n'.format(smoothing_mode)

import math
EPS = 0.001
ECO_DELTA = 0.005
SPORT_DELTA = 0.01
velocity_command = Twist()
velocity_input = Twist()

BUMPER_HIT = False

# CALLBACK: smooths velocity publishes it and state value to kobuki. happens every 10ms
def smooth_and_publish(data):
	global velocity_pub, velocity_command, velocity_input, config_command, SPORT_DELTA, ECO_DELTA, EPS, BUMPER_HIT

	# CONFIGURE KOBUKI BASED ON CONFIG_COMMAND
	# publish smoothing mode to LED 1
	# smoothing and publishing velocity
	def in_tol(curr, target):
		return math.fabs(curr - target) < EPS

	ebrake_on = config_command.data[3] == 1
	bumper_onlybackwards_enabled = config_command.data[1] == 1
        bumper_autostop_enabled = config_command.data[0] == 1

    # BUMPER_HIT is mutated by bumper event via a callback
	if ebrake_on or (BUMPER_HIT and bumper_autostop_enabled): 
		velocity_command.linear.x = 0
		velocity_command.angular.z = 0
		velocity_pub.publish(velocity_command)
		return

	# smooth the linear velo
	not_smoothing = config_command.data[4] == 0
	if not_smoothing:
		velocity_command.linear.x = velocity_input.linear.x 
	elif not in_tol(velocity_command.linear.x, velocity_input.linear.x):
		smoothing_mode = config_command.data[4]
		delta = ECO_DELTA if smoothing_mode == 1 else SPORT_DELTA
		if velocity_command.linear.x > velocity_input.linear.x:
			velocity_command.linear.x -= delta
		elif velocity_command.linear.x < velocity_input.linear.x:
			velocity_command.linear.x += delta

	# don't smooth angular velo
	velocity_command.angular.z = velocity_input.angular.z

	going_straight_backwards = velocity_command.linear.x < 0 and math.fabs(velocity_command.angular.z < 0.7)

	# if we've bumped into something and we aren't going backwards, then publish
	if going_straight_backwards or (not BUMPER_HIT and not bumper_onlybackwards_enabled):
		velocity_pub.publish(velocity_command)

# BACKUP BEEPING AND BLINKING
BACKUP_LED_ON = False # backup LED is LED 2
GREEN, OFF, BEEP_NOISE = 1, 0, 4
LED_MSG = Led()

def backup_control(data):
	global config_command, BACKUP_LED_ON, LED_MSG, BUMPER_HIT
	global velocity_command, velocity_input

	currently_backing_up = velocity_input.linear.x < 0
	not_turning = math.fabs(velocity_input.angular.z) < 0.7

	# if we're backing up, then we're forgiven of the sin of bumping into something...
	if currently_backing_up and not_turning:
		BUMPER_HIT = False

	backup_signal_enabled = config_command.data[2] == 1

	if not backup_signal_enabled or not currently_backing_up:
		return

	curr_color = OFF if BACKUP_LED_ON else GREEN
	BACKUP_LED_ON = not BACKUP_LED_ON

	LED_MSG.value = curr_color
	led2_pub.publish(LED_MSG)
	sound_pub.publish(BEEP_NOISE)

def bumper_control(data):
	global BUMPER_HIT, config_command
	# e.g.: `left_bumper_hit = data.bumper == 0 and data.state == 1`

	bumper_hit_event = data.state == 1 # if any bumper is hit
	bumper_control_enabled = config_command.data[0] == 1
	if bumper_hit_event and bumper_control_enabled: 
		print 'SETTING BUMPER HIT TO TRUE'
		BUMPER_HIT = True

# INIT SUBSCRIBERS
rospy.init_node('smoother', anonymous=True)
rospy.Subscriber('robot_twist', Twist, twist_callback) 
rospy.Subscriber('robot_commands', Int32MultiArray, command_callback)
rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_control)

# takes published values $(SMOOTHING_RATE) closer to input values, publishes every 10ms (0.01s)
if __name__ == '__main__':
	# calls smooth_and_publish every 0.01s
	SMOOTHER_DURATION_S = 0.01 # seconds: 0.01s == 10ms
	BLINKER_DURATION_S = 0.25
	try:
		publish_thread = rospy.Timer(rospy.Duration(SMOOTHER_DURATION_S), smooth_and_publish)
		backup_thread = rospy.Timer(rospy.Duration(BLINKER_DURATION_S), backup_control)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
