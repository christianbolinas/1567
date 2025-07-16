#!/usr/bin/env python

'''
TODO: 

+ stop once we reach the end of the line
+ maybe smooth z(t) towards z(t+1)...? the description says to not use a smoother node, but we can smooth in the file...
'''

import roslib
import rospy
import copy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob
from time import time
from geometry_msgs.msg import Twist

# ---------- CONFIG ---------- #
LINEAR_X = 0.2
K_p = 0.004
K_i = 0.008
K_d = 0.015
# ---------------------------- #

colorImage = Image()
isColorImageReady = False
velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
velocity_command = Twist()
blob_arr = []

HALFWAY_ACROSS_FOV = 640 // 2

def updateColorImage(data): # Callback for `/v4l/camera/image_raw`.
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

def update_blobs_info(data): # Callback for `/blobs`.
    global blob_arr
    blob_arr = data.blobs
    num_blobs = data.blob_count

def get_error(): 
    global HALFWAY_ACROSS_FOV, blob_arr

    def get_center():
        # return max(blob_arr, key=lambda blob: blob.area).x # use biggest blob
        # n_biggest = sorted(blob_arr, key=lambda blob: blob.area, reverse=True)[:N]

        min_x = min(blob_arr, key=lambda blob: blob.left).left
        max_x = max(blob_arr, key=lambda blob: blob.right).right
        return (min_x + max_x) // 2

    if len(blob_arr) == 0:
        return None

    blob_center = get_center()
    error = HALFWAY_ACROSS_FOV - blob_center
    return error

prev_error, total_error, prev_time = 0.0, 0.0, time()

def main():
    global colorImage, isColorImageReady, velocity_pub, velocity_command, blob_arr, LINEAR_X
    global center_x, HALFWAY_ACROSS_FOV, K_p, K_i, K_d, prev_error, total_error, prev_time

    rospy.init_node("showBlobs", anonymous=True)
    rospy.Subscriber('/blobs', Blobs, update_blobs_info) 
    rospy.Subscriber("/v4l/camera/image_raw", Image, updateColorImage)
    bridge = CvBridge()
    cv2.namedWindow("Blob Locations")
    velocity_command.linear.x = LINEAR_X 
    most_recent_blob_seen = time()

    while not rospy.is_shutdown() and not isColorImageReady: 
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"

        start_time = time()
        # --- PID CALCULATION ---
        curr_time = time()
        time_diff = curr_time - prev_time                                       # dt
        prev_time = curr_time                                               
        curr_error = get_error()                                                # e(t)
        if len(blob_arr) == 0:
            velocity_command.angular.z = 0
        else:
            most_recent_blob_seen = time()
            # draw a rectangle between furthest left and furthest right
            THELEFT = min(blob_arr, key=lambda blob: blob.left).left
            THERIGHT = max(blob_arr, key=lambda blob: blob.right).right

            THETOP = 270
            THEBOTTOM = 210
            cv2.rectangle(color_image, (THELEFT, THETOP), (THERIGHT, THEBOTTOM), (0,255,0), 2)
            # that's done

            # calculate PID, publish
            total_error += curr_error * time_diff                               # \int_0^t [...]
            diff_error = curr_error - prev_error                                # de/dt
            prev_error = curr_error
            v_t = K_p * curr_error + K_i * total_error + K_d * diff_error
            velocity_command.angular.z = v_t
            velocity_command.linear.x = LINEAR_X

        no_blobs_timeout = time() - most_recent_blob_seen > 3
        print 'time: {}, |blobs| = {}'.format(time() - start_time, len(blob_arr))
        if len(blob_arr) == 0 and no_blobs_timeout:
            velocity_command.linear.x = 0
            velocity_pub.publish(velocity_command)
            break

        velocity_pub.publish(velocity_command)
        cv2.imshow("Blob Locations", color_image)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break;

cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
