##this file takes information from the keyboard and cuts through remotectrl to talk to smoother

import rospy
from std_msgs.msg import Float32


start_x = 0
start_y = 0
start_deg = 0
current_angular_distance = 0
current_linear_distance = 0

def determine_distance(data):

	global start_x, start_y, start_deg, current_linear_distance, current_angular_distance

	q = [data.pose.pose.orientation.x,
	data.pose.pose.orientation.y,
	data.pose.pose.orientation.z,
	data.pose.pose.orientation.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	# roll, pitch, and yaw are in radian
	degree = yaw * 180 / math.pi
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y

	
	if(start_x == 0 and start_y == 0): #i.e. not already updated
		start_x = x
		start_y = y
	else:
		current_linear_distance = math.sqrt( (x - start_x)**2 + (y - start_y)**2 ) 

	if(start_deg == 0):
		start_deg = degree
	else:
		current_angular_distance = start_deg - degree
	
	return current_linear_distance, currentr_angular_distance
	

def messenger():
    
    while not rospy.is_shutdown():
	
	#we are expecting two floats. one the maximum speed 
	val = int(input('Enter 0 for linear movement and 1 for angular movement > ')
	speed = float(input('Enter maximum speed  > ')) 
	distance = float(input('Enter distance/degree  > ')) 
    


if __name__ == '__main__':
    try:
	messenger()
    except rospy.ROSInterruptException:
        pass


