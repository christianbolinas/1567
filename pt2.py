#!/usr/bin/env python

#-------------------------------------------------------
import math
import rospy
import roslib
import copy
import cv2

from std_msgs.msg import Float32, Int32MultiArray, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cmvision.msg import Blobs, Blob
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from time import time

#--------------------------------------------------------

#---------------------------------------------------------
#
#start robot left most side facing right 
#	turn left until either ball/goal in view 
#		- if ball in center, stop and record angle |-> ball_alpha
#		- if goal in center, stop and record angle |-> goal_alpha
#	turn right back to face right. go straight until the far right side
#	turn to face left 
#
#----------------------------------------------------------


#---------------------------------------------------------------------------------

reset_pub = rospy.Publisher("/mobile_base/commands/reset_odometry", Empty, queue_size=10)
velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)

odometry = Odometry()
command = Twist()
colorImage = Image()
blobsInfo = Blobs()
isColorImageReady = False

velocity_command = Twist()
blob_arr = []
ball_arr = []
goal_arr = []


HALFWAY_ACROSS_FOV = 640 // 2

#constants to determine movement of kobuki
start_x = 0.0
start_y = 0.0
start_deg = 0.0
current_angular_distance = 0.0
current_linear_distance = 0.0

#constants for calculation of movements
ball_alpha = 0.0
goal_alpha = 0.0
ball_beta = 0.0
goal_beta = 0.0
known_distance = 0.0 #TO BE CHANGED LATER 

change_x = 0.0
change_y = 0.0
target_x = 0.0
target_y = 0.0


#----------------------------HELPER FUNCTIONS----------------------

def goal_center():
    '''
    The goal is defined as the center of the yellow blob that itself is inside of pink.
    Returns x-coordinate of the goal if it's recognized, or `None` if the goal isn't in frame.
    '''
    yellow_blobs = filter(lambda blob: blob.color == 'yellow', blob_arr)
    if not yellow_blobs:
        return None
    biggest_yellow_blob_center = max(yellow_blobs, key=lambda blob: blob.size).x
    # now check that `biggest_yellow_blob_center` is inside of pink
    pink_blobs = filter(lambda blob: blob.color == 'pink', blob_arr)
    if not pink_blobs:
        return None
    leftmost_pink = min(pink_blobs, key=lambda blob: blob.left).left
    rightmost_pink = max(pink_blobs, key=lambda blob: blob.right).right
    goal_found = leftmost_pink < biggest_yellow_blob_center and biggest_yellow_blob_center < rightmost_pink
    if not goal_found:
        return None

    return biggest_yellow_blob_center

def charge(theta, distance):
	#TODO, actual movement of robot for kicking 

def calc_angles():
	global ball_alpa, goal_alpha, ball_beta, goal_beta, known_distance
	global change_x, change_y, target_x, target_y

	#alpha and betas all in degrees 
	ball_alpha = math.radians(ball_alpha)
	ball_beta = math.radians(ball_beta)
	goal_alpha = math.radians(goal_alpha)
	goal_beta = math.radians(goal_beta)

	goal_x = known_distance + (known_distance * math.tan(goal_beta)) / (math.tan(goal_alpha))
	ball_x = known_distance + (known_distance * math.tan(ball_beta)) / (math.tan(ball_alpha))

	goal_h = goal_x * math.tan(goal_alpha)
	ball_h = ball_x * math.tan(ball_alpha)

	m = (goal_h - ball_h)/(goal_x - ball_x)

	#calculate change and target
	change_x = math.sqrt((known_distance * known_distance)/(m*m + 1))
	change_y = m *change_x

	target_x = ball_x + change_x
	target_y = ball_y + change_y

	#route
	cur_y = 0
	cur_x = 0 #TODO get current placement x? if displacement, 0? 
	delta_x = tar_x - cur_x
	#delta_y?
	d = (tar_x - cur_x)*(tar_x - cur_x) + (tar_y - cur_y)*(tar_y - cur_y)

	theta = math.arctan(delta_y/delta_x)

	#turn robot to theta, go forward d distance, stop, turn till ball center frame, go forward full speed .75
	charge(theta, distance)


#TODO: add colors into colors.txt
def filter_red(blob):
	return blob.name == "Red"

def filter_orange():
	return blob.name == "Orange"

def updateColorImage(data): # Callback for `/v4l/camera/image_raw`.
	global colorImage, isColorImageReady
	colorImage = data
	isColorImageReady = True

def update_blobs_info(data): # Callback for `/blobs`.
	global blob_arr, ball_arr, goal_arr
	blobsInfo = data
	blob_arr = data.blobs
	
	if blob_arr:
		print "blob!"
		print blob_arr[0]
	#here we split into red and orange blobs 
	ball_arr = filter(filter_red, blob_arr)
	goal_arr = filter(filter_orange, blob_arr)
    
	num_blobs = data.blob_count

def red_blob_boundaries():
	global ball_arr, ONLY_USE_BIGGEST_BLOB, BY_AREA

	if ball_arr is None or len(ball_arr) == 0: # python short-circuits
		return None, None, None, None

	elif ONLY_USE_BIGGEST_BLOB: 
		new_blob_arr = copy.deepcopy(ball_arr) 
		biggest_blob = max(ball_arr, key=BY_AREA)
		ball_arr = [biggest_blob]
		return biggest_blob.left, biggest_blob.right, biggest_blob.bottom, biggest_blob.top

	else:
		min_x, max_x, min_y, max_y = 10000, -10000, 10000, -10000 # identity elements for < and > operators
		for blob in ball_arr:
			if blob.left < min_x:
				min_x = blob.left
			if blob.right > max_x:
				max_x = blob.right
			if blob.top > max_y:
				max_y = blob.top
			if blob.bottom < min_y:
				min_y = blob.bottom
		return min_x, max_x, min_y, max_y

def orange_blob_boundaries():
	global goal_arr, ONLY_USE_BIGGEST_BLOB, BY_AREA

	if goal_arr is None or len(goal_arr) == 0: # python short-circuits
		return None, None, None, None

	elif ONLY_USE_BIGGEST_BLOB: 
		new_blob_arr = copy.deepcopy(goal_arr) 
		biggest_blob = max(goal_arr, key=BY_AREA)
		goal_arr = [biggest_blob]
		return biggest_blob.left, biggest_blob.right, biggest_blob.bottom, biggest_blob.top

	else:
		min_x, max_x, min_y, max_y = 10000, -10000, 10000, -10000 # identity elements for < and > operators
		for blob in goal_arr:
			if blob.left < min_x:
				min_x = blob.left
			if blob.right > max_x:
				max_x = blob.right
			if blob.top > max_y:
				max_y = blob.top
			if blob.bottom < min_y:
				min_y = blob.bottom
		return min_x, max_x, min_y, max_y



def odom(data):
	global start_x, start_y, start_deg, current_linear_distance, current_angular_distance

	# Convert quaternion to degree
	q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	# roll, pitch, and yaw are in radian
	degree = math.fabs(yaw * 180 / math.pi)
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y

	#if(start_x == 0 and start_y == 0): #i.e. not already updated
		#start_x = x
		#start_y = y
	#else:
	current_linear_distance = math.sqrt( x**2 + y**2 ) #sometimes need to change to +=??????

	#print "distance = {} and x: {} and y {}".format(current_linear_distance, x, y)

	#if(start_deg == 0):
		#start_deg = degree
	
	current_angular_distance = degree 
	

def resetOdom():
	pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
	pub.publish(Empty())

#---------------------------------------------------------------------------------------
#following from christian's friend instructions
def main():
	global isColorImageReady, velocity_pub, velocity_command, blob_arr, ball_arr, goal_arr, LINEAR_X, colorImage
	global center_x, HALFWAY_ACROSS_FOV, current_angular_distance, start_deg, known_distance

	turn1 = 0 #= +1 when found ball, +2 when found goal, (=3 when found both)
	turn2 = 4 #= +1 when found ball, +2 when found goal, (=3 when found both), set to zero when turn1 done

	rospy.init_node("showBlobs", anonymous=True)
	rospy.Subscriber('/odom', Odometry, odom)
	rospy.Subscriber('/blobs', Blobs, update_blobs_info) 
	rospy.Subscriber("/v4l/camera/image_raw", Image, updateColorImage)
	bridge = CvBridge()
	cv2.namedWindow("Blob Locations")

	#odometer starts acting funny for angles greater than 180 or 120 or whatever, so we reset odometer every 90 degree
	#but keep track!
	total_angle = 0 

	while not rospy.is_shutdown() and not isColorImageReady: 
		pass
	
	while not rospy.is_shutdown():

		try:
			color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8") 
		except CvBridgeError, e:
			print e
			print "ball/goal"
			
		if turn1 < 3:
			#turn left slowly until either ball or goal is in view 
			
			#check if exceeding odom limits
			if current_angular_distance >= 90:
				total_angle += current_angular_distance
				resetOdom()
				start_deg = 0.0
				current_angular_distance = 0.0

			velocity_command.angular.z = .1 
			velocity_pub.publish(velocity_command)

		if turn2 < 3: 
			#TODO: implement to turn right back to face right (keep track of total_angle again)
			#TODO: move to far left, make this known_distance, set as like .8 m? or something 
			#TODO: turn to face left (two turns of angle 90)
			
			total_angle = 0
			#turn right slowly until either ball or goal is in view 
			print "turning!"
			#check if exceeding odom limits
			if current_angular_distance >= 90:
				total_angle += current_angular_distance
				resetOdom()
				start_deg = 0.0
				current_angular_distance = 0.0

			velocity_command.angular.z = -.1 
			velocity_pub.publish(velocity_command)


		goal_left, goal_right, goal_bottom, goal_top = orange_blob_boundaries()
		goal_recognized = goal_left is not None

		ball_left, ball_right, ball_bottom, ball_top = red_blob_boundaries()
		ball_recognized = ball_left is not None


		if ball_recognized and (turn1 == 2 or turn1 == 0): #eg only goal or nothing found 
			#record angle for ball in ball_alpha, increment turn1 by 1 
			ball_alpha = total_angle
			rospy.sleep(1.0)
			print ball_alpha
                         
		if goal_recognized and (turn1 == 1 or turn1 == 0): #eg only goal or nothing found
            #record angle for ball in ball_alpha, increment turn1 by 1
			goal_alpha = total_angle
			print goal_alpha
		
		if turn1 > 2: #both found, change to turn2. also reset odometer to keep track of angle
			turn2 = 0
			print "Resetting!"
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0


		if ball_recognized and (turn2 == 2 or turn2 == 0): #eg only goal or nothing found 
			#record angle for ball in ball_alpha, increment turn1 by 1 
			ball_beta = total_angle
			print ball_beta
                         
		if goal_recognized and (turn2 == 1 or turn2 == 0): #eg only goal or nothing found
            #record angle for ball in ball_alpha, increment turn1 by 1
			goal_beta = total_angle
			print goal_beta

		if turn2 == 3:
			calc_angles()
			#break out loop 
				
		cv2.imshow("Blob Locations", color_image)

        
		key = cv2.waitKey(1)
		if key == ord('q'):
			break;

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()


