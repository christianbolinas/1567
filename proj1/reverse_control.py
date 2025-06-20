#!/usr/bin/env python

''' Christian 6/18
reverse_control.py: blinks and beeps when backing up.

SUBSCRIBES: 
- `/mobile_base/commands/velocity`: receives commands of type Twist

PUBLISHES: 
- `/mobile_base/commands/led1`: commands of type Led
- `/mobile_base/commands/led2`: commands of type Led
- `/mobile_base/commands/sound`: commands of type Sound
=======================================================

- TODODODODO STILL HAVE TO MAKE IT SO THIS ONLY HAPPENS WHEN THAT OPTION IS TOGGLED.
'''

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led, Sound

# CONFIG GLOBALS
TICK_DURATION = 0.25 # seconds
GREEN, OFF, BEEP_NOISE = 1, 0, 4

# INIT GLOBALS
rospy.init_node("backup_blinker", anonymous=True)
led1_pub = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=1)
led2_pub = rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=1)
sound_pub = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=1)
led_msg = Led()
sound_msg = Sound()

# STATE GLOBALS
BACKING_UP = False
LEDS_ON = False 

# callback for blink_timer; called once every TICK_DURATION seconds.
def blink_timer_callback(data):
	global BACKING_UP, LEDS_ON, led_msg, GREEN, OFF

	if not BACKING_UP: 
		return

	COLOR = GREEN if LEDS_ON else OFF
	LEDS_ON = not LEDS_ON
	led_msg.value = COLOR
	led1_pub.publish(led_msg)
	led2_pub.publish(led_msg)

def beep_timer_callback(data):
	global BACKING_UP, sound_msg

	if not BACKING_UP: 
		return
	
	# play beep
	sound_pub.publish(BEEP_NOISE)

# callback for velocity change event: if backing up, blink LEDs
def backup_callback(data):
	global BACKING_UP
	BACKING_UP = data.linear.x < 0
	
def reverse_controller():
	global led_msg

	# subscribe to kobuki velocity, which is what we're publishing to (to make it move around). 
	# upon velocity event, call backup_callback.
	rospy.Subscriber('/mobile_base/commands/velocity', Twist, backup_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		blink_timer = rospy.Timer(rospy.Duration(TICK_DURATION), blink_timer_callback)
		beep_timer = rospy.Timer(rospy.Duration(TICK_DURATION), beep_timer_callback)
		reverse_controller()
	except rospy.ROSInterruptException: # turn off LEDs on progn exit lol
		OFF = 0 
		led_msg.value = OFF
		led1_pub.publish(led_msg)
		led2_pub.publish(led_msg)
