#!/usr/bin/env python

import sys
import math
import rospy
import roslib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# ----- CONFIG ----- 
robot_speed = 0.2 # m/s
lookahead_distance = 0.15 # m

# ----- GLOBALS AND ROSPY BOILERPLATE ----- 
velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
velocity_command = Twist()

robot_x, robot_y, robot_theta = None, None, None

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

    # I'm sure e_f_q is Very Slow and I know calculating in a callback (bc it blocks) is Very Bad but we ball
    roll, pitch, yaw = euler_from_quaternion(q) 
    robot_x = data.pose.pose.position.x
    robot_y = data.pose.pose.position.y
    robot_theta = yaw

# ----- MEAT AND POTATOES -----

class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def parse_coordinate(line):
    coords = line.split(',')
    x = float(coords[0])
    y = float(coords[1])
    coord = Coordinate(x, y)
    return coord

def pure_pursuit(robot_speed, robot_x, robot_y, robot_theta, future_points, lookahead_distance):
    '''
    ROBOT_THETA AND THE RETURN VALUE ARE IN RADIANS, AS ARE ALL INTERNAL CALCULATIONS.

    Takes in robot state, config vars (linear speed, lookahead distance), and points to follow, and returns new angle to
    turn towards.

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
        for point in future_points:
            dx, dy = point.x - robot_x, point.y - robot_y
            dist = math.sqrt(dx**2 + dy**2)
            if dist >= lookahead_distance:
                return point
            return future_points[-1]

    target_point = lookahead_point()
    dx, dy = target_point.x - robot_x, target_point.y - robot_y
    angular_error = math.atan2(dy, dx) # don't ask me to remember any analytic geometry. I don't.
    alpha = normalize_angle(angular_error - robot_theta)
    thetaprime = 2 * robot_speed * math.sin(alpha) / lookahead_distance
    return thetaprime

if __name__ == '__main__':
    # THESE AREN'T NEEDED: THIS CONDITIONAL IS IN SCOPE
    # global velocity_pub, velocity_command
    # global robot_speed, lookahead_distance, robot_x, robot_y, robot_theta

    rospy.init_node('coordinate_follower', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    if len(sys.argv) != 2:
        print 'usage: ./coordinate_follower.py <FILE-TO-FOLLOW>.txt'
        exit(0)

    fname = sys.argv[1] # the file to read movement instrs from
    with open(fname) as f:
        lines = f.readlines()

    future_points = [parse_coordinate(line) for line in lines]
    last_point = future_points[-1]

    while True: 
        velocity_command.angular.z = pure_pursuit(robot_speed, robot_x, robot_y, robot_theta, future_points,
            lookahead_distance)
        velocity_command.linear.x = robot_speed
        velocity_pub.publish(velocity_command)

        # stopping condition
        STOPPING_THRESHOLD = 0.05 # m
        dstop_x = last_point.x
        dstop_y = last_point.y
        dstop = math.sqrt(dstop_x**2 + dstop_y**2)
        if dstop <= STOPPING_THRESHOLD:
            velocity_command.linear.x = 0.0
            velocity_command.angular.z = 0.0
            velocity_pub.publish(velocity_command)
            break
