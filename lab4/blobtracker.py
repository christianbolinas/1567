#!/usr/bin/env python

import rospy
import roslib
import cv2
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob

blob_arr = None

def get_blobs_callback(data):
	global blob_arr
	blob_arr = data.blobs

def biggest_blob_naive():
	'''
	Returns center of biggest blob by finding biggest recognized blob and returning its center.
	'''
	global blob_arr
	if blob_arr == None:
		return None
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
        return None
    else: #T(blob_arr) = 4|blob_arr|, but could be done T(blob_arr) = |blob_arr|... does it matter?
        lowest_x = min(blob_arr, key=lambda blob: blob.x)
        highest_x = max(blob_arr, key=lambda blob: blob.x)
        lowest_y = min(blob_arr, key=lambda blob: blob.y)
        highest_y = max(blob_arr, key=lambda blob: blob.y)

        center_x = highest_x - lowest_x
        center_y = highest_y - lowest_y
        return center_x, center_y

def turn_until_blob_centered(blob_x, blob_y):
    raise NotImplementedError # TODO

if __name__ == '__main__':
    # subscribe to image stream
	rospy.Subscriber("blobs", Blobs, get_blobs_callback)

	while not rospy.is_shutdown():
        try:
            blob_x, blob_y = biggest_blob()
            turn_until_blob_centered(blob_x, blob_y)
        except ROSInterruptException:
            pass
