#!/usr/bin/env python3
#
#   plan_driver.py
#
#   Converts current position and desired position into velocity/twist
#
#   Node:       /plan_driver
#   Publish:    /vel_cmd          geometry_msgs/Twist
#   Subscribe:  /odom                   Odometry
#               /move_base_simple/goal  geometry_msgs/PoseStamped
#               /scan                   sensor_msgs/LaserScan
#
import math
import rospy

from geometry_msgs.msg  import Point, Quaternion, Twist
from geometry_msgs.msg  import PoseStamped
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import LaserScan


#
#   Simple Driver
#
class DriverObj():
    # Constants for goto function
    CLOSE_ENOUGH_DISTANCE = 0.02 # m
    CLOSE_ENOUGH_THETA = 0.2 # rad
    OMEGA_P_CONST_SPIN = 1.5
    OMEGA_I_CONST = 0.001
    MAX_VEL = 0.75 # m/s
    VEL_TIME_CONST = 2.0 # m/s
    OMEGA_CONST = 0.9

    # constants for scannin'
    ESTOP_OFFSET_MULT = 3.0 # 1/s(DANGER ZONE)
    ESTOP_OFFSET_MIN = 0.2 # m

    def __init__(self):
        ## define class variables
        # velocity/angle change to publish
        self.vx = 0
        self.wz = 0
        # current
        self.cur_x = 0   # m
        self.cur_y = 0   # m
        self.cur_t = 0   # rad
        # desired
        self.des_x = 0   # m
        self.des_y = 0   # m
        self.des_t = 0 # rad

        self.simple_err = 0

        ## initalize node
        # Create a publisher to send velocity commands.
        self.pub_velcmd = rospy.Publisher('/vel_cmd', Twist, queue_size=10)

        # Create a subscriber to listen to odometry commands commands.
        rospy.Subscriber('/odom', Odometry, self.cb_update_current)

        # Create a subscriber to listen to pose commands
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_update_desired)

        # Create subscriber to listen to laser scan
        rospy.Subscriber('/scan', LaserScan, self.cb_update_scan)

    # car driver protocol
    def gotoPoint(self):
        # distance to travel
        dx = self.des_x - self.cur_x
        dy = self.des_y - self.cur_y
        d = math.sqrt(dx*dx + dy*dy)
        targett = math.atan2(dy, dx)
        relt = angleDiff(self.des_t, self.cur_t)

        # relt = angleDiff(self.des_t, self.cur_t)
        vx = clamp((1/self.VEL_TIME_CONST)*d*math.cos(targett - self.cur_t), -self.MAX_VEL, self.MAX_VEL)
        wz = self.OMEGA_CONST*math.sin(targett - self.cur_t)

        if (d < self.CLOSE_ENOUGH_DISTANCE):
            vx = 0
            self.simple_err += relt
            wz = self.OMEGA_P_CONST_SPIN * relt + self.OMEGA_I_CONST*self.simple_err

        if (abs(relt) < self.CLOSE_ENOUGH_THETA):
            self.simple_err = 0
            wz = 0

        # save message
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz
        return msg


    '''CALLBACK FUNCTIONS'''
    # Callback for /move_base_simple/goal
    def cb_update_desired(self, msg):
        # Update desired x, y, theta, sin/cos
        self.des_x = msg.pose.position.x
        self.des_y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.des_t = 2*math.atan2(qz, qw)


    # Callback for /odom
    def cb_update_current(self, msg):
        # Update current x, y, theta, sin/cos
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.cur_t = 2*math.atan2(qz, qw)

        msg_velcmd = self.gotoPoint()
        self.pub_velcmd.publish(msg_velcmd)

    # Callback for /scan
    def cb_update_scan(self, msg):
        # stop if something is right in front
        range_min = msg.range_min
        range_max = msg.range_max

        if (self.vx < 0):
            # can't see anything!
            return
        
        offset = max(self.ESTOP_OFFSET_MULT*self.vx, self.ESTOP_OFFSET_MIN)

        for r in msg.ranges:
            # throw out anything outside of the range
            if (r < range_min) or (r > range_max):
                continue
            if (r < (range_min + offset)):
                self.vx = 0
                self.des_x = self.cur_x
                self.des_y = self.cur_y
                self.des_t = self.cur_t
                break


# Compute the angle difference
def angleDiff(t1, t2):
    return (t1-t2) - 2.0*math.pi * round(0.5*(t1-t2)/math.pi)


# Clamp a variable between a maximum and minimum (Saturated)
def clamp(x, min_x, max_x):
    return max(min(max_x, x), min_x)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('plan_driver')

    # Instantiate the Driver object
    simpledriver = DriverObj()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Driver spinning...")
    rospy.spin()
    rospy.loginfo("Driver stopped.")
