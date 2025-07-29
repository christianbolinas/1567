#!/usr/bin/env python

import sys
import os
import math
from time import time
import rospy
import roslib
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# ----- CONFIG ----- 
robot_speed = 0.2 # m/s
lookahead_distance = 0.15 # m
eps = 0.08 # m

# ----- GLOBALS AND ROSPY BOILERPLATE ----- 
velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
velocity_command = Twist()

robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0

def odom_callback(data):
    '''
    Sets global robot_x, robot_y, and robot_theta. I heart mutating global state.

    Adapted from Dr. Tan (that is, I changed from camelCase to snake_case, and didn't convert to radians)...
    '''
    global robot_x, robot_y, robot_theta

    q = [data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w]

    # I'm sure e_f_q is Very Slow and I know calculating in a callback is Very Bad (bc it blocks)
    # ...but this keeps the complexity nicely encapsulated right here, so we ball
    roll, pitch, yaw = euler_from_quaternion(q) 
    robot_x = data.pose.pose.position.x
    robot_y = data.pose.pose.position.y
    robot_theta = yaw

reset_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
def reset_odom():
    while reset_pub.get_num_connections() == 0:
        pass
    reset_pub.publish(Empty())

# ----- MEAT AND POTATOES -----

class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Cylinder:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

def pure_pursuit(robot_speed, robot_x, robot_y, robot_theta, future_points, lookahead_distance):
    '''
    ROBOT_THETA AND THE RETURN VALUE ARE IN RADIANS, AS ARE ALL INTERNAL CALCULATIONS.

    Takes in robot state, config vars (linear speed, lookahead distance), and points to follow, and returns new angle to
    turn towards. See design doc for explanation and intuition. 

    ADAPTED FROM: https://github.com/xiaoxiae/PurePursuitAlgorithm/blob/master/src/main/PurePursuit.java (the main
    thing) and https://stackoverflow.com/a/2323034 (for angle normalization).
    '''
    def normalize_angle(angle):
        twopi = math.pi * 2
        reduced = angle % twopi
        positive_remainder = (reduced + twopi) % twopi
        in_range = positive_remainder - twopi if positive_remainder > math.pi else positive_remainder
        return in_range

    def lookahead_point():
        return future_points[0]

    target_point = lookahead_point()
    dx, dy = target_point.x - robot_x, target_point.y - robot_y
    angular_error = math.atan2(dy, dx) 
    alpha = normalize_angle(angular_error - robot_theta)
    omega = 2 * robot_speed * math.sin(alpha) / lookahead_distance
    return omega

def parse_cylinder(fname):
    with open(fname) as f:
        lines = f.readlines()
    ''' this is what the file will look like:
    # cylinder at (3,0) with radius 0.25; target at (5,0)
    3,0,0.25
    5,0
    '''
    _ = lines[0] # this is just a comment for metadata
    cylinder_info = lines[1].split(',')
    target_info = lines[2].split(',')
    cylinder_x, cylinder_y, cylinder_r = float(cylinder_info[0]), float(cylinder_info[1]), float(cylinder_info[2])
    the_cylinder = Cylinder(Coordinate(cylinder_x, cylinder_y), cylinder_r)
    the_target = Coordinate(float(target_info[0]), float(target_info[1]))
    return the_target, the_cylinder

def get_detour(cylinder):
    kobuki_radius = 0.175
    eps = 0.08

    dist_cylinder_center = math.sqrt(cylinder.center.x**2 + cylinder.center.y**2) # robot starts at origin
    # slope of perpendicular line = negative reciprocal of line. line is from (0,0) to (c.c.x, c.c.y).
    # no need to calculate dx and dy, they're already there

    perpendicular_x = -cylinder.center.y / dist_cylinder_center
    perpendicular_y = cylinder.center.x / dist_cylinder_center

    dist_detour = cylinder.radius + kobuki_radius + eps

    detour_x = cylinder.center.x + perpendicular_x * dist_detour
    detour_y = cylinder.center.y + perpendicular_y * dist_detour

    detour_point = Coordinate(detour_x, detour_y)
    return detour_point

if __name__ == '__main__':
    # PYTHON MAIN HAS ACCESS TO ALL GLOBAL VARS IN SCOPE. KEEPING COMMENTS SO WE KNOW WHAT GLOBALS ARE USED
    # global velocity_pub, velocity_command, robot_speed, lookahead_distance, robot_x, robot_y, robot_theta

    rospy.init_node('coordinate_follower', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    reset_odom()

    if len(sys.argv) != 2:
        print 'usage: ./cylinder_avoider.py <FILE-TO-FOLLOW>.txt (see cylinder-1.txt for example)'
        exit(0)

    target, cylinder = parse_cylinder(sys.argv[1])

    print "target: ({},{})".format(target.x, target.y)
    print "cylinder: ({},{}), r={}".format(cylinder.center.x, cylinder.center.y, cylinder.radius)

    detour_point = get_detour(cylinder)

    print "detour point @ ({}, {})".format(detour_point.x, detour_point.y)

    future_points = [detour_point, target]
    last_point = future_points[-1]
    
    start_time = time()

    while True: 
        # log bc shit doesn't play nice
        os.system('clear')
        print "robot: ({},{}), target: ({},{})".format(robot_x, robot_y, future_points[0].x, future_points[0].y)
        velocity_command.angular.z = pure_pursuit(robot_speed, robot_x, robot_y, robot_theta, future_points,
            lookahead_distance)
        velocity_command.linear.x = robot_speed
        velocity_pub.publish(velocity_command)

        in_tol = abs(robot_x - future_points[0].x) < eps and abs(robot_y - future_points[0].y) < eps
        if in_tol: 
            if len(future_points) == 1:
                print "done!"
                exit(0)
                break # LOL unnecessary
            else:
                future_points.pop(0)
                print "going to {}".format(future_points[0])

        if time() - start_time > 30: # seconds
            print "Timed out. Exiting now."
            exit(0)
