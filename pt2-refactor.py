#!/usr/bin/env python

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

'''
PROCEDURE:
    start robot left most side facing right 
        turn left until either ball/goal in view 
            - if ball in center, stop and record angle |-> ball_alpha
            - if goal in center, stop and record angle |-> goal_alpha
        turn right back to face right. go straight until the far right side
            turn to face left 
'''

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

start_x = 0.0
start_y = 0.0
start_deg = 0.0
current_angular_distance = 0.0
current_linear_distance = 0.0

ball_alpha = 0.0
goal_alpha = 0.0
ball_beta = 0.0
goal_beta = 0.0
known_distance = 0.5 # half meter, change if necessary 

change_x = 0.0
change_y = 0.0
target_x = 0.0
target_y = 0.0

def goal_center():
    global blob_arr
    '''
    The goal is defined as the center of the yellow blob that itself is inside of pink.
    Returns x-coordinate of the goal if it's recognized, or `None` if the goal isn't in frame.

    TODO make sure this works
    '''
    yellow_blobs = filter(lambda blob: blob.color == 'yellow', blob_arr)
    if not yellow_blobs:
        return None
    biggest_yellow_blob_center = max(yellow_blobs, key=lambda blob: blob.size).x
    
    pink_blobs = filter(lambda blob: blob.color == 'pink', blob_arr)
    if not pink_blobs:
        return None
    leftmost_pink = min(pink_blobs, key=lambda blob: blob.left).left
    rightmost_pink = max(pink_blobs, key=lambda blob: blob.right).right
    goal_found = leftmost_pink < biggest_yellow_blob_center and biggest_yellow_blob_center < rightmost_pink
    if not goal_found:
        return None

    return biggest_yellow_blob_center

def charge(x):
	'''
    actual movement of robot for kicking 
 	'''
	global blob_arr, velocity_command, velocity_pub
	resetOdom()
	start_deg = 0.0
	current_angular_distance = 0.0
	current_linear_distance = 0.0 

	while current_linear_distance < x:
		velocity_command.linear.x = 0.25
		velocity_pub.publish(velocity_command)
		print x
	velocity_command.linear.x = 0
	velocity_pub.publish(velocity_command)

	#turn until ball is in center view 
	blob_centered = False
	biggest_blue_blob_x = max([blob for blob in blob_arr if blob.color == "Blue"], key=lambda blob: blob.area).x
	while not blob_centered:
		screen_center = 640 // 2
		tol = 20
		if abs(biggest_blue_blob_x - screen_center) < tol:
			blob_centered = True
			break # paranoia

		# actually turn
		velocity_command.angular.z = -.5
		velocity_pub.publish(velocity_command)

	resetOdom()
	start_deg = 0.0
	current_angular_distance = 0.0
	current_linear_distance = 0.0 

	while current_linear_distance <= 1:
		velocity_command.linear.x = 0.8
		velocity_pub.publish(velocity_command)
	velocity_command.linear.x = 0
	velocity_pub.publish(velocity_command)
	
	
def calc_angles():
	global ball_alpha, goal_alpha, ball_beta, goal_beta, known_distance

	#find tangent of angles to find ball location 
	m1 = math.tan(math.radians(ball_alpha))
	m2 = math.tan(math.radians(ball_beta))

	#find location of ball
	ball_x = -(known_distance * m2)/(m1 - m2)
	ball_y = m1 * ball_x 

	#find tangent of angles to find goal location 
	m1 = math.tan(math.radians(goal_alpha))
	m2 = math.tan(math.radians(goal_beta))

	#find location of goal
	goal_x = (known_distance * m2)/(m1 - m2)
	goal_y = m1 * goal_x 


	print ball_x, ball_y
	print goal_x, goal_y

	#calculate trajectory. draw a line between goal and ball: y = mx + b
	m = (ball_y - goal_y)/(ball_x - goal_x)
	b = ball_y - (m * ball_x)
	
	#find point on line where y = 0, ie only move robot left or right 
	x = (-b)/m
	print(x)


	charge(x)

def updateColorImage(data): # Callback for `/v4l/camera/image_raw`.
	global colorImage, isColorImageReady
	colorImage = data
	isColorImageReady = True

def update_blobs_info(data): # Callback for `/blobs`.
	global blob_arr, ball_arr, goal_arr
	blobsInfo = data
	blob_arr = data.blobs
	
	# here we split into blue and yellow blobs 
	ball_arr = filter(lambda blob: blob.name == "Blue", blob_arr)
	goal_arr = filter(lambda blob: blob.name == "Yellow", blob_arr)
    
	num_blobs = data.blob_count

def blue_blob_boundaries():
	global ball_arr, BY_AREA
    # global ONLY_USE_BIGGEST_BLOB

	if ball_arr is None or len(ball_arr) == 0: # python short-circuits
		return None, None, None, None

	# elif ONLY_USE_BIGGEST_BLOB: 
	# 	new_blob_arr = copy.deepcopy(ball_arr) 
	# 	biggest_blob = max(ball_arr, key=BY_AREA)
	# 	ball_arr = [biggest_blob]
	# 	return biggest_blob.left, biggest_blob.right, biggest_blob.bottom, biggest_blob.top

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

def yellow_blob_boundaries():
	global goal_arr, BY_AREA
    # global ONLY_USE_BIGGEST_BLOB,

	if goal_arr is None or len(goal_arr) == 0: # python short-circuits
		return None, None, None, None

	# elif ONLY_USE_BIGGEST_BLOB: 
	# 	new_blob_arr = copy.deepcopy(goal_arr) 
	# 	biggest_blob = max(goal_arr, key=BY_AREA)
	# 	goal_arr = [biggest_blob]
	# 	return biggest_blob.left, biggest_blob.right, biggest_blob.bottom, biggest_blob.top

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
	global isColorImageReady, velocity_pub, velocity_command, blob_arr, ball_arr, goal_arr, LINEAR_X, colorImage, ball_alpha
	global center_x, HALFWAY_ACROSS_FOV, current_angular_distance, start_deg, known_distance, current_linear_distance, ball_beta, goal_beta, goal_alpha

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

	resetOdom()
	start_deg = 0.0
	current_angular_distance = 0.0


	while not rospy.is_shutdown() and not isColorImageReady: 
		pass
	
	while not rospy.is_shutdown():

		try:
			color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8") 
		except CvBridgeError, e:
			print e
			print "ball/goal"
			

		#-----------------turn one starts at the left, turning left until both ball and goal is found 
		if turn1 < 3 and turn1 != 4:
			#turn left slowly until either ball or goal is in view 
			
			#check if exceeding odom limits
			if current_angular_distance >= 90:
				total_angle += current_angular_distance
				resetOdom()
				start_deg = 0.0
				current_angular_distance = 0.0

			velocity_command.angular.z = -.5
			velocity_pub.publish(velocity_command)
		#-------------------------------------------------------------

		#---------------turn two starts at the right, turning left until both ball and goal is found 
		if turn2 < 3 and turn2 != 4: 
			
			#turn right slowly until either ball or goal is in view 
			#print "turning!"
			#check if exceeding odom limits

			if current_angular_distance >= 90:
				total_angle += current_angular_distance
				resetOdom()
				start_deg = 0.0
				current_angular_distance = 0.0

			velocity_command.angular.z = -.5 
			velocity_pub.publish(velocity_command)

		#-------------------------------------------------------------

		#these track whether the goal and ball has been spotted by the robot 

                screen_center = 640 // 2
                tol = 20
		goal_left, goal_right, goal_bottom, goal_top = yellow_blob_boundaries()
		goal_recognized = goal_left is not None
                '''
                if goal_left is None:
                    goal_recognized = False
                else:
                    goal_center = (goal_left + goal_right) / 2
                    goal_recognized = abs(goal_center - screen_center) < tol
                '''
		
		ball_left, ball_right, ball_bottom, ball_top = blue_blob_boundaries()
		# ball_recognized = ball_left is not None
                if ball_left is None:
                    ball_recognized = False
                else:
                    ball_center = (ball_left + ball_right) / 2
                    ball_recognized = abs(ball_center - screen_center) < tol

		#-------------------------------------------------------------

		#-----------------------------trackers for the turning
		if ball_recognized and (turn1 == 2 or turn1 == 0): #eg only goal or nothing found 
			#record angle for ball in ball_alpha, increment turn1 by 1 
			total_angle += current_angular_distance
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0

			print "Ball recognized"
			ball_alpha = total_angle
			turn1 += 1
			rospy.sleep(1.0)
			print ball_alpha
                         
		if goal_recognized and (turn1 == 1 or turn1 == 0): #eg only goal or nothing found
            #record angle for ball in ball_alpha, increment turn1 by 2
			total_angle += current_angular_distance
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0


			goal_alpha = total_angle
			print "goal recognized!" 
			rospy.sleep(1.0)
			turn1 += 2
			print goal_alpha

		if ball_recognized and (turn2 == 2 or turn2 == 0): #eg only goal or nothing found 
			#record angle for ball in ball_beta, increment turn2 by 1 
			total_angle += current_angular_distance
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0


			print "ball recognized pt 2!"
			ball_beta = total_angle
			print ball_beta
			turn2 += 1
                         
		if goal_recognized and (turn2 == 1 or turn2 == 0): #eg only goal or nothing found
            #record angle for goal in goal_beta, increment turn2 by 2
			total_angle += current_angular_distance
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0


			print "goal recognized pt 2!"
			goal_beta = total_angle
			print goal_beta
			turn2 +=2

		#----------------------------------------------------------------------

		#-----------------------------------------------------------------
		# the first turn is done, so we turn back to starting point, move to the right side and turn to left, and start turn 3

		if turn1 == 3: #both found, change to turn2. also reset odometer to keep track of angle
			turn2 = 0
			print "turn 1 done!"
			ball_recognized = False
			goal_recognized = False
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0
			velocity_command.angular.z = 0 #no need to turn anomore 
			velocity_pub.publish(velocity_command)


			#turn right back to face right (keep track of total_angle again)
			print "turning back this many degrees"
			print total_angle
			num_90_turns = (int)(total_angle / 90)
			total_angle = total_angle - (num_90_turns * 90) #whatever is left over
			print num_90_turns
			i = 0
			while i < num_90_turns: #turn the other way in groupings of 90 degrees 
				while current_angular_distance <= 90:
					velocity_command.angular.z = .5 #turning back around 	
					velocity_pub.publish(velocity_command)
				resetOdom()
				start_deg = 0.0
				current_angular_distance = 0.0
				i += 1 
				print "one 90 degree turn done!"

			print "turning remainder!"
			print total_angle
			while current_angular_distance <= total_angle:
				velocity_command.angular.z = .5 #turning back around 	
				velocity_pub.publish(velocity_command)
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0
			current_linear_distance = 0.0 #as a just in case 
			total_angle = 0
			velocity_command.angular.z = 0 	
			velocity_pub.publish(velocity_command)

			
			#move to far right, make this known_distance, set as like .8 m? or something 
			while current_linear_distance <= .5:
				velocity_command.linear.x = -0.25
				velocity_pub.publish(velocity_command)
			velocity_command.linear.x = 0
			velocity_pub.publish(velocity_command)
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0
			current_linear_distance = 0.0
			
			turn1 = 4 #turn turn1 off
			continue
		#--------------------------------------------------------------------------

		if turn2 == 3:
			
			print "turn 2 done!"
			velocity_command.angular.z = 0 
			velocity_pub.publish(velocity_command)
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0

			#turn right back to face right (keep track of total_angle again)
			print "turning back this many degrees"
			print total_angle
			num_90_turns = (int)(total_angle / 90)
			total_angle = total_angle - (num_90_turns * 90) #whatever is left over
			print num_90_turns
			i = 0
			while i < num_90_turns: #turn the other way in groupings of 90 degrees 
				while current_angular_distance <= 90:
					velocity_command.angular.z = .5 #turning back around 	
					velocity_pub.publish(velocity_command)
				resetOdom()
				start_deg = 0.0
				current_angular_distance = 0.0
				i += 1 
				print "one 90 degree turn done!"

			print "turning remainder!"
			print total_angle
			while current_angular_distance <= total_angle:
				velocity_command.angular.z = .5 #turning back around 	
				velocity_pub.publish(velocity_command)
			resetOdom()
			start_deg = 0.0
			current_angular_distance = 0.0
			current_linear_distance = 0.0 #as a just in case 
			total_angle = 0
			velocity_command.angular.z = 0 	
			velocity_pub.publish(velocity_command)

			calc_angles()
			break
				
		cv2.imshow("Blob Locations", color_image)

        
		key = cv2.waitKey(1)
		if key == ord('q'):
			break;

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
