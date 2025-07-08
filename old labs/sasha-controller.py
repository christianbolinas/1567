#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float32, Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)

reset_pub = rospy.Publisher("/mobile_base/commands/reset_odometry", Empty, queue_size=10)

odometry = Odometry()
command = Twist()

#constants to determine movement of kobuki
start_x = 0.0
start_y = 0.0
start_deg = 0.0
current_angular_distance = 0.0
current_linear_distance = 0.0

def commandParse(commands):
	# running through all commands in sets of three. every three is one command
	global current_linear_distance, current_angular_distance
		
	#while True:
		#continue

	i = 0
	while i < len(commands) - 1: 
		resetOdom()
		start_x = 0.0
		start_y = 0.0
		start_deg = 0.0
		current_angular_distance = 0.0
		current_linear_distance = 0.0

		type_movement = commands[i] #1/2 for linear movement, 3/4 for angular movement
		i += 1
		speed = commands[i]
		i += 1
		distance = commands[i] #or degree
		i +=1


		if type_movement == 1 or type_movement == 2: #linear
			moveLinear(type_movement, speed, distance)
		elif type_movement == 3 or type_movement == 4: #angular

			## to deal with desired angular distance over 120 degrees, we break up movements into smaller ones summing up to total
			while distance > 90:
				moveAngular(type_movement, speed, 90)
				distance = distance - 90
				print "piecewise turn of {} degree done, {} left ".format(90, distance)
				resetOdom()
				start_x = 0.0
				start_y = 0.0
				start_deg = 0.0
				current_angular_distance = 0.0
				current_linear_distance = 0.0

			print "breaking out of loop!"
			moveAngular(type_movement, speed, distance)
		
		print "movement done!"
		resetOdom()

def getUserInput():
	commands = []
	isdone = False
	
	while not isdone:
		command_input = int(raw_input("Enter command (0 to submit): "))

		if command_input == 0:
			isdone = True
			continue
		else:
			commands.append(command_input)

		speed = float(raw_input("Enter maximum speed : "))
		distance = float(raw_input("Enter distance/degree : "))

		commands.append(speed)
		commands.append(distance)

	print commands
	commandParse(commands)

def moveLinear(direction, speed, distance): #1 for forwards, 2 for backwards
	global velocity_pub, command, current_linear_distance # we expect this to be maintained by odom 

	print "moving direction = {}, speed = {}, distance = {}".format(direction, speed, distance) 
	if direction == 1:
		command.linear.x = speed
	else:
		command.linear.x = -speed
	while current_linear_distance < distance:
		velocity_pub.publish(command)
		#print "moved = {} to target distanmce {}".format(current_linear_distance, distance) 
	
	#command done, stop the robot
	command.linear.x = 0.0
	velocity_pub.publish(command)

def moveAngular(direction, speed, degree): #negative turn right, positive turn left, 3 right, 4 left\
	global velocity_pub, command, start_deg, current_angular_distance

	print "moving direction = {}, speed = {}, distance = {}".format(direction, speed, degree) 
	# angular goes from -1 to 1, so adjust degree

	#print degree
	#print current_angular_distance

	if direction == 3:
		command.angular.z = -speed
	elif direction == 4:
		command.angular.z = speed

	while current_angular_distance < degree:
		velocity_pub.publish(command)
		#print "moving direction = {}".format(current_angular_distance) 
	
	#command done, stop the robot
	command.angular.z = 0.0
	velocity_pub.publish(command)



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

def cleanUp():
	global velocity_pub, command
	command.linear.x = 0.0
	command.angular.z = 0.0
	velocity_pub.publish(command)
	rospy.sleep(1)

def controller():
	rospy.init_node("controller", anonymous=True)
	rospy.Subscriber('/odom', Odometry, odom)
	rospy.on_shutdown(cleanUp)
	
	while velocity_pub.get_num_connections() == 0 or reset_pub.get_num_connections() == 0:
		pass

	while not rospy.is_shutdown():
		getUserInput()

	rospy.spin()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		pass
