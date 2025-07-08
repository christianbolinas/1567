#!/usr/bin/env python

import rospy
import roslib
import cv2
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob
from geometry_msgs.msg import Twist

blob_arr = None

IMAGE_MAX_V = 479
IMAGE_MAX_X = 639
DEADZONE_SIZE = 100

velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)

def get_blobs_callback(data):
	global blob_arr
	blob_arr = data.blobs
#	print("Got: " + blob_arr)

def biggest_blob_naive():
	'''
	Returns center of biggest blob by finding biggest recognized blob and returning its center.
	'''
	global blob_arr
	if blob_arr == None:
		return None, None
	else:
		# equiv. to `def blob_area(blob):
			# return (blob.right - blob.left) * (blob.top - blob.bottom)`
		blob_area = lambda blob: (blob.right - blob.left) * (blob.top - blob.bottom)
		biggest_blob = max(blob_arr, key=blob_area)
		return biggest_blob.x, biggest_blob.y

def biggest_blob():
	'''
	Different from Dr. Tan's, which is "blob center is weighted average of blob centers."
	This is "blob center is center of all-encompassing blob."
	'''
	global blob_arr
	if blob_arr == None:
		return None, None
	else: #T(blob_arr) = 4|blob_arr|, but could be done T(blob_arr) = |blob_arr|... does it matter?
		lowest_x = min(blob_arr, key=lambda blob: blob.x)
		highest_x = max(blob_arr, key=lambda blob: blob.x)
		lowest_y = min(blob_arr, key=lambda blob: blob.y)
		highest_y = max(blob_arr, key=lambda blob: blob.y)

		center_x = highest_x - lowest_x
		center_y = highest_y - lowest_y
		return center_x, center_y

def turn_until_blob_centered(blob_x, blob_y):
	center_x = IMAGE_MAX_X / 2
	zone_half = DEADZONE_SIZE / 2
	velocity = 0

	command = Twist()

	if center_x - zone_half < blob_x < center_x + zone_half:
		return

	velocity = blob_x - center_x

	if velocity > 0:
		velocity -= zone_half
	elif velocity < 0:
		velocity += zone_half

	velocity /= center_x - zone_half

	command.angular.z = velocity
	velocity_pub.publish(command)

if __name__ == '__main__':
	# subscribe to image stream
	rospy.init_node("blob_tracker", anonymous=True)
	rospy.Subscriber("/blobs", Blobs, get_blobs_callback)

	while not rospy.is_shutdown():


		blob_x, blob_y = biggest_blob()

		if blob_x == None:
			pass

		turn_until_blob_centered(blob_x, blob_y)
