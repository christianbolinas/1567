#!/usr/bin/env python

''' 
Lab 4 pt. 3 assignment:
=======================

Modify program to eliminate false blob detection. We're going to look for a blob of pink inside blue. 
For simplicity, we're only going to turn towards a small =BrightPink= blob inside of a larger =Blue= blob.
'''

# ---------- CONFIG ---------- #
TOL_DEG = 45
TURN_SPEED = 1

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

def blob_boundaries():
    '''We're only returning coordinates if we have a small =BrightPink= inside of a larger =Blue=, and those are the two largest blobs.'''
    global blob_arr

    if blob_arr is None or len(blob_arr) < 2: # python short-circuits
        return None, None, None, None

    blues = filter(lambda blob: blob.name == 'Blue', blob_arr)

    if len(blues) == 0:
        return None, None, None, None

    leftmost_blue = min(blues, key=lambda blob: blob.left).left
    rightmost_blue = max(blues, key=lambda blob: blob.right).right
    lowest_blue = min(blues, key=lambda blob: blob.bottom).bottom
    highest_blue = max(blues, key=lambda blob: blob.top).top

    pink = max(blob_arr, key=lambda blob: blob.area)
    biggest_is_pink = pink.name == 'BrightPink'
    if not biggest_is_pink:
        # print 'biggest is not pink.'
        return None, None, None, None
    
    # print 'biggest is pink' 

    print 'blue x ranges from {} to {}, pink.x == {}'.format(leftmost_blue, rightmost_blue, pink.x)
    pink_in_blue_x = pink.x > leftmost_blue and pink.x < rightmost_blue
    pink_in_blue_y = pink.y > lowest_blue and pink.y < highest_blue
    pink_in_blue = pink_in_blue_x and pink_in_blue_y
    if pink_in_blue:
        print 'pink is in blue!'

    return pink.left, pink.right, pink.bottom, pink.top

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

        object_left, object_right, object_bottom, object_top = blob_boundaries()
        blob_recognized = object_left is not None

        if blob_recognized:
            # turns towards largest blob.
            # defines the blob, if exists, as biggest blob. otherwise, draws rectangle around all colors of interest and turns.
            center_x = (object_left + object_right) // 2
            cv2.rectangle(color_image, (object_left, object_top), (object_right, object_bottom), (0, 255, 0), 2)
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
