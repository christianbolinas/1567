#!/usr/bin/env python

# TODO ACTUALLY TEST ALL THIS IT DEF DOESN'T WORK - Christian 7/14

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
from time import time

# ---------- CONFIG ---------- #
LINEAR_X = 0.2
K_p = 1
K_i = 1
K_d = 1
# ---------------------------- #

colorImage = Image()
isColorImageReady = False
velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
velocity_command = Twist()
velocity_command.linear.x = LINEAR_X
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

def get_error(): # TODO
    global HALFWAY_ACROSS_FOV, blob_arr

    if len(blob_arr) == 0: 
        return

    furthest_left_px = min(blob_arr, key=lambda blob: blob.left)
    furthest_right_px = max(blob_arr, key=lambda blob: blob.right)
    blob_center = furthest_right_px - furthest_left_px
    error = HALFWAY_ACROSS_FOV - blob_center
    return error

prev_error, total_error, prev_time = 0.0, 0.0, time()

def main():
    global colorImage, isColorImageReady, velocity_pub, velocity_command, blob_arr
    global center_x, HALFWAY_ACROSS_FOV, K_p, K_i, K_d, prev_error, total_error, prev_time

    rospy.init_node("showBlobs", anonymous=True)
    rospy.Subscriber('/blobs', Blobs, update_blobs_info) 
    rospy.Subscriber("/v4l/camera/image_raw", Image, updateColorImage)
    bridge = CvBridge()
    cv2.namedWindow("Blob Locations")

    while not rospy.is_shutdown() and not isColorImageReady and len(blob_arr) == 0:
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"

        # --- PID CALCULATION ---
        curr_time = time()
        time_diff = curr_time - prev_time                                   # dt
        prev_time = curr_time                                               
        curr_error = get_error()                                            # e(t)
        total_error += curr_error * time_diff                               # \int_0^t [...]
        diff_error = curr_error - prev_error                                # de/dt
        prev_error = curr_error
        new_z = K_p * curr_error + K_i * total_error + K_d * diff_error     # pid
        velocity_command.angular.z = new_z
        velocity_pub.publish(velocity_command)
        cv2.imshow("Blob Locations", color_image)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break;

cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
