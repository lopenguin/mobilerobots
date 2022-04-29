#!/usr/bin/env python3
#
#   simpledriver.py
#
#   Converts current position and desired position into velocity/twist
#
#   Node:       /driver
#   Publish:    /vel_cmd          geometry_msgs/Twist
#   Subscribe:  /odom                   Odometry
#               /move_base_simple/goal  geometry_msgs/PoseStamped
#
import math
import rospy

from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg      import Odometry


#
#   Simple Driver
#
class DriverObj():
    CLOSE_ENOUGH_DISTANCE = 0.02 # m
    CLOSE_ENOUGH_THETA = 0.2 # rad
    OMEGA_P_CONST_SPIN = 1.5
    OMEGA_P_CONST_DRIVE = 0.8 # 1/s
    OMEGA_I_CONST = 0.001
    VEL_P_CONST = 0.5 # 1/s
    MAX_VEL = 0.25 # m/s

    VEL_TIME_CONST = 2.0 # m/s
    OMEGA_CONST = 0.9


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

    ## GOTO function options
    # new version of lame goto that uses p control on orientation and velocity
    def lameGoto2(self):
        # distance to travel
        dx = self.des_x - self.cur_x
        dy = self.des_y - self.cur_y
        d = math.sqrt(dx*dx + dy*dy)

        if (d < self.CLOSE_ENOUGH_DISTANCE):
            # MODE: TARGET POSITION REACHED, TURN TO FACE GOAL
            # # compute relative angle to travel (q_des q_cur*)
            # relqw = self.des_qw*self.cur_qw + self.des_qz*self.cur_qz   # sin(relt/2)
            # relqz = self.des_qz*self.cur_qw - self.des_qw*self.cur_qz   # cos(relt/2)
            # relt = 2*math.atan2(relqz, relqw)
            relt = angleDiff(self.des_t, self.cur_t)

            # travel!
            vx = 0
            if (abs(relt) < self.CLOSE_ENOUGH_THETA):
                self.simple_err = 0
                wz = 0
            else:
                self.simple_err += relt
                wz = self.OMEGA_P_CONST_SPIN * relt + self.OMEGA_I_CONST*self.simple_err
                print(relt)
                print(self.simple_err)

        else:
            # MODE: TURN TO FACE TARGET AND MOVE
            # compute relative angle of target
            targett = math.atan2(dy, dx)
            # tqw = math.cos(targett/2)
            # tqz = math.sin(targett/2)
            # relqw = tqw*self.cur_qw + tqz*self.cur_qz   # sin(relt/2)
            # relqz = tqz*self.cur_qw - tqw*self.cur_qz   # cos(relt/2)
            # relt = 2*math.atan2(relqz, relqw)
            relt = angleDiff(targett, self.cur_t)

            if (relt < self.CLOSE_ENOUGH_THETA):
                vx = min(self.VEL_P_CONST * d, self.MAX_VEL)
            else:
                vx = 0

            # travel!
            # vx = min(self.VEL_P_CONST * d, self.MAX_VEL)
            wz = self.OMEGA_P_CONST_DRIVE*relt

        # save message
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz
        return msg

    # car driver protocol
    def easyGoto(self):
        # distance to travel
        dx = self.des_x - self.cur_x
        dy = self.des_y - self.cur_y
        d = math.sqrt(dx*dx + dy*dy)
        targett = math.atan2(dy, dx)

        # relt = angleDiff(self.des_t, self.cur_t)
        vx = (1/self.VEL_TIME_CONST/2)*d*math.cos(targett - self.cur_t)
        wz = self.OMEGA_CONST*math.sin(targett - self.cur_t)

        if (d < self.CLOSE_ENOUGH_DISTANCE):
            vx = 0

        relt = angleDiff(self.des_t, self.cur_t)
        if (abs(relt) < self.CLOSE_ENOUGH_THETA):
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


    # Callback for /move_base_simple/goal, saves x, y, theta desired to class
    def cb_update_desired(self, msg):
        # Update desired x, y, theta, sin/cos
        self.des_x = msg.pose.position.x
        self.des_y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.des_t = 2*math.atan2(qz, qw)


    # Callback for /odom, saves x, y, and theta current to class
    def cb_update_current(self, msg):
        # Update current x, y, theta, sin/cos
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.cur_t = 2*math.atan2(qz, qw)

        # msg_velcmd = self.lameGoto2()
        msg_velcmd = self.easyGoto()
        self.pub_velcmd.publish(msg_velcmd)

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
    rospy.init_node('driver')

    # Instantiate the Driver object
    simpledriver = DriverObj()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Driver spinning...")
    rospy.spin()
    rospy.loginfo("Driver stopped.")
