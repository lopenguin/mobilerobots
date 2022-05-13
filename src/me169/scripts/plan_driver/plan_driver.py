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
from sensor_msgs.msg    import LaserScan


#
#   Simple Driver
#
class DriverObj():
    # Constants for goto function
    CLOSE_ENOUGH_DISTANCE = 0.02 # m
    CLOSE_ENOUGH_THETA = 0.2 # rad
    OMEGA_P_CONST_SPIN = 1.0
    OMEGA_I_CONST = 0.001
    MAX_VEL = 0.6 # m/s
    VEL_TIME_CONST = 2.0 # m/s
    OMEGA_CONST = 0.5
    FORWARD_ONLY_DIST = 1 # m
    GO_FORWARD_THETA = 0.75 # rad

    # constants for scannin'
    ESTOP_OFFSET_MULT = 0.85 # 1/s(DANGER ZONE)
    ESTOP_OFFSET_MIN = 0.05 # m

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
        self.stopped = False

        ## initalize node
        # Create a publisher to send velocity commands.
        self.pub_velcmd = rospy.Publisher('/vel_cmd', Twist, queue_size=10)

        # Create a subscriber to listen to pose commands.
        rospy.Subscriber('/pose', PoseStamped, self.cb_update_current)

        # Create a subscriber to listen to pose commands from rviz
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_update_desired)

        # Create subscriber to listen to laser scan
        rospy.Subscriber('/scan', LaserScan, self.cb_update_scan)

    # car driver protocol
    # def gotoPoint(self):
    #     # distance to travel
    #     dx = self.des_x - self.cur_x
    #     dy = self.des_y - self.cur_y
    #     d = math.sqrt(dx*dx + dy*dy)
    #     targett = math.atan2(dy, dx)
    #     relt = angleDiff(self.des_t, self.cur_t)

    #     # relt = angleDiff(self.des_t, self.cur_t)
    #     self.wz = self.OMEGA_CONST*math.sin(targett - self.cur_t)
    #     self.vx = clamp((1/self.VEL_TIME_CONST)*d*math.cos(targett - self.cur_t), -self.MAX_VEL, self.MAX_VEL)
    #     # if (d > self.FORWARD_ONLY_DIST):
    #     #     self.vx = clamp(self.vx, 0, self.MAX_VEL)
    #     #     self.wz = self.OMEGA_CONST*(targett - self.cur_t)

    #     if (d < self.CLOSE_ENOUGH_DISTANCE):
    #         self.vx = 0
    #         self.simple_err += relt
    #         self.wz = self.OMEGA_P_CONST_SPIN * relt + self.OMEGA_I_CONST*self.simple_err

    #     if (abs(relt) < self.CLOSE_ENOUGH_THETA):
    #         self.simple_err = 0
    #         self.wz = 0

    #     # save message
    #     msg = Twist()
    #     msg.linear.x = self.vx
    #     msg.linear.y = 0.0
    #     msg.linear.z = 0.0
    #     msg.angular.x = 0.0
    #     msg.angular.y = 0.0
    #     msg.angular.z = self.wz
    #     return msg

    # simpler version
    def gotoPoint(self):
        # precompute travel distance
        dx = self.des_x - self.cur_x
        dy = self.des_y - self.cur_y
        d = math.sqrt(dx*dx + dy*dy)    # distance to travel
        targett = math.atan2(dy, dx)    # angle to face target

        # initialize velocity and angle to 0
        self.wz = 0
        self.vx = 0

        # velocity: move if far away and kinda facing target
        ttogo = angleDiff(targett, self.cur_t)
        if (d > self.CLOSE_ENOUGH_DISTANCE) and (abs(ttogo) < self.GO_FORWARD_THETA):
            self.vx = clamp((1/self.VEL_TIME_CONST)*d*math.cos(ttogo), 0, self.MAX_VEL)

        # theta: move if (a) far away and not facing target or (b) close and not facing desired
        relt = angleDiff(self.des_t, self.cur_t)
        if (d > self.CLOSE_ENOUGH_DISTANCE) and (abs(ttogo) > self.CLOSE_ENOUGH_THETA):
            self.wz = self.OMEGA_CONST*ttogo
            self.simple_err = 0
        elif (d < self.CLOSE_ENOUGH_DISTANCE) and (abs(relt) > self.CLOSE_ENOUGH_THETA):
            self.simple_err += relt
            self.wz = self.OMEGA_P_CONST_SPIN*relt + self.OMEGA_I_CONST*self.simple_err
        # save message
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.wz
        return msg


    '''CALLBACK FUNCTIONS'''
    # Callback for /move_base_simple/goal
    def cb_update_desired(self, msg):
        assert (msg.header.frame_id == 'map'), "Message not in map frame"
        # Update desired x, y, theta, sin/cos
        self.des_x = msg.pose.position.x
        self.des_y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.des_t = 2*math.atan2(qz, qw)
        self.stopped = False


    # Callback for /odom
    def cb_update_current(self, msg):
        assert (msg.header.frame_id == 'map'), "Message not in map frame"
        # Update current x, y, theta, sin/cos
        self.cur_x = msg.pose.position.x
        self.cur_y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.cur_t = 2*math.atan2(qz, qw)

        if (self.stopped):
            # bypass the goto
            msg_velcmd = Twist()
            msg_velcmd.linear.x = self.vx
            msg_velcmd.linear.y = 0.0
            msg_velcmd.linear.z = 0.0
            msg_velcmd.angular.x = 0.0
            msg_velcmd.angular.y = 0.0
            msg_velcmd.angular.z = self.wz
        else:
            msg_velcmd = self.gotoPoint()
        self.pub_velcmd.publish(msg_velcmd)

    # Callback for /scan
    def cb_update_scan(self, msg):
        # stop if something is right in front
        range_min = msg.range_min
        range_max = msg.range_max

        if (self.vx < 0.0):
            # can't see anything!
            return
        
        offset = max(self.ESTOP_OFFSET_MULT*self.vx, self.ESTOP_OFFSET_MIN)

        for r in msg.ranges:
            # throw out anything outside of the range
            if (r < range_min) or (r > range_max):
                continue
            if (r < (range_min + offset)):
                self.stopped = True
                self.vx = 0
                self.wz = 0
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
