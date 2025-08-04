#!/usr/bin/env python

''' Ethan 7/23
odom_recorder.py: records a series of points as the robot moves

SUBSCRIBES:
- `odom`: odometry data of type `Odometry`
- `joy`: controller input of type `Joy`

PUBLISHES:
- `reset_odometry`: resets the robot odometry when an `Empty` is published
'''

import rospy
import math
import os
import sys
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy


reset_pub = rospy.Publisher("/mobile_base/commands/reset_odometry", Empty, queue_size=10)

point_list = []
recording_enabled = False
file_name = ""
interval = 0.1 # The distance travelled before recording another point

def odom_callback(data):
	global point_list, recording_enabled, interval
	if not recording_enabled:
		return

	# Setup recording when enabled
	if len(point_list) == 0:
		point_list.append((0, 0))
		reset_pub.publish(Empty())

	current_position = (data.pose.pose.position.x, data.pose.pose.position.y)
	last_point = point_list[-1]

	# Calculate the distance from the last point
	distance = math.hypot(
		current_position[0] - last_point[0],
		current_position[1] - last_point[1]
	)

	if distance >= interval:
		point_list.append(current_position)
		print(f"Point recorded: {current_position}")

def joystickCallback(data):
	global recording_enabled
	a_val = data.buttons[0]

	if a_val > 0:
		if recording_enabled:
			recording_enabled = False
			print("Recording stopped.")
			end_recording()
		else:
			recording_enabled = True
			print("Recording started.")

def end_recording():
	global point_list
	
	os.makedirs('recordings', exist_ok=True)

	with open('recordings/' + file_name, 'w') as file:
		for i in point_list:
			file.write(str(i[0]) + ',' + str(i[1]) + '\n')

	rospy.signal_shutdown("Recording finished")

def recorder():
	global file_name
	file_name = str(raw_input("Enter output file name: "))

	rospy.init_node("odom_recorder", anonymous=True)
	rospy.Subscriber('/odom', Odometry, odom_callback)
	rospy.Subscriber("joy", Joy, joystickCallback)
	# rospy.on_shutdown(cleanUp)
	
	while reset_pub.get_num_connections() == 0 and not rospy.is_shutdown():
		rospy.sleep(0.1)

	rospy.spin()

if __name__ == '__main__':
	try:
		recorder()
	except rospy.ROSInterruptException:
		pass