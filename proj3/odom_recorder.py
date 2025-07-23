#!/usr/bin/env python

''' Ethan 7/23
odom_recorder.py: records a series of points as the robot moves

SUBSCRIBES:
- `odom`: odometry data of type `Odometry`

PUBLISHES:
- `reset_odometry`: resets the robot odometry when an `Empty` is published
'''

import rospy
import math
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

reset_pub = rospy.Publisher("/mobile_base/commands/reset_odometry", Empty, queue_size=10)

point_list = []
recording_enabled = False
interval = 0.1 # The distance travelled before recording another point

def odom(data):
	if not recording_enabled:
		return

	# Setup recording when enabled
	if point_list.len == 0:
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
		point_list.append((current_position))

def recorder():
	rospy.init_node("odom_recorder", anonymous=True)
	rospy.Subscriber('/odom', Odometry, odom)
	rospy.on_shutdown(cleanUp)
	
	while velocity_pub.get_num_connections() == 0 or reset_pub.get_num_connections() == 0:
		pass

	while not rospy.is_shutdown():
		getUserInput()

	rospy.spin()

if __name__ == '__main__':
	try:
		recorder()
	except rospy.ROSInterruptException:
		pass