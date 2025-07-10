#!/usr/bin/env python

''' 
Lab 4 pts. 1, 2 assignment:
===========================

1. Merge overlapped blobs. I did this, but I didn't use Dr. Tan's algorithm.
2. Make robot turn towards object at constant speed (linear.x = 0.0, angular.z = 0.1). 

NOTE: This angular.z value results in no movement. It works as expected if we use angular.z = 1. 
'''

# ---------- CONFIG ---------- #
TOL_DEG = 45
TURN_SPEED = 1
ONLY_USE_BIGGEST_BLOB = False

# ---------- IMPORTS ---------- #
import roslib
import rospy
import copy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob
from time import time
from geometry_msgs.msg import Twist
import copy


# ---------- ALGORITHM(S) ---------- #
blob_arr = None
BY_AREA = lambda blob: blob.area

def blob_boundaries():
    global blob_arr, ONLY_USE_BIGGEST_BLOB, BY_AREA

    if blob_arr is None or len(blob_arr) == 0: # python short-circuits
        return None, None, None, None

    elif ONLY_USE_BIGGEST_BLOB: 
        new_blob_arr = copy.deepcopy(blob_arr) 
        biggest_blob = max(blob_arr, key=BY_AREA)
        blob_arr = [biggest_blob]
        return biggest_blob.left, biggest_blob.right, biggest_blob.bottom, biggest_blob.top

    else:
        min_x, max_x, min_y, max_y = 10000, -10000, 10000, -10000 # identity elements for < and > operators
        for blob in blob_arr:
            if blob.left < min_x:
                min_x = blob.left
            if blob.right > max_x:
                max_x = blob.right
            if blob.top > max_y:
                max_y = blob.top
            if blob.bottom < min_y:
                min_y = blob.bottom
        return min_x, max_x, min_y, max_y

def tan_blob_center():
    '''
    Dr. Tan's "find biggest blob" algorithm. Blob centroid is weighted (by blob area) average of blob centers.
    I can't figure out if this works or not. No clue what it's doing.
    '''
    global blob_arr
    if blob_arr is None or len(blob_arr) == 0: # python short-circuits
        return

    x = 0
    y = 0
    area = 0
    for box in blob_arr:
        area += box.area
        x += box.x * box.area
        y += box.y * box.area
        x /= area
        y /= area
    
    if x != 0:
        print 'Dr. Tan\'s center: ({}, {})'.format(x, y)
    return x, y


# ---------- MOVEMENT ---------- #

colorImage = Image()
isColorImageReady = False
velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
velocity_command = Twist()

HALFWAY_ACROSS_FOV = 640 // 2
center_x = None

def turn_towards():
    global velocity_pub, velocity_command, HALFWAY_ACROSS_FOV, TOL_DEG, center_x

    if center_x is None:
        velocity_command.angular.z = 0
        return

    in_tol = abs(HALFWAY_ACROSS_FOV - center_x) <= TOL_DEG
    if in_tol:
        velocity_command.angular.z = 0
        return
    
    should_turn_left = center_x < HALFWAY_ACROSS_FOV
    
    if should_turn_left: 
        velocity_command.angular.z = TURN_SPEED
    else:
        velocity_command.angular.z = -TURN_SPEED
        
colorImage = Image()
isColorImageReady = False

def updateColorImage(data):
    '''
    Callback for `/v4l/camera/image_raw`.
    '''
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

def update_blobs_info(data): 
    '''
    Callback for `/blobs`.
    '''
    global blob_arr
    blob_arr = data.blobs

    num_blobs = data.blob_count


def main():
    global colorImage, isColorImageReady, center_x, velocity_pub

    rospy.init_node("showBlobs", anonymous=True)
    rospy.Subscriber('/blobs', Blobs, update_blobs_info) 
    rospy.Subscriber("/v4l/camera/image_raw", Image, updateColorImage)
    bridge = CvBridge()
    cv2.namedWindow("Blob Locations")

    while not rospy.is_shutdown() and not isColorImageReady:
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"

        christian_left, christian_right, christian_bottom, christian_top = blob_boundaries()
        blob_recognized = christian_left is not None

        if blob_recognized:
            # turns towards largest blob.
            # defines the blob, if exists, as biggest blob. otherwise, draws rectangle around all colors of interest and turns.
            center_x = (christian_left + christian_right) // 2
            cv2.rectangle(color_image, (christian_left, christian_top), (christian_right, christian_bottom), (0, 255, 0), 2)
        else:
            center_x = None

        turn_towards() # changes velocity_command to point towards center_x (center of object)
        velocity_pub.publish(velocity_command)
        cv2.imshow("Blob Locations", color_image)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break;

cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
