#!/usr/bin/env python3
#
#   cardriver.py
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
import numpy as np

from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg      import Odometry






## GOTO function options
class LocalPlan:
    def __init__(self, fromState, toState):
        # Compute the connection.
        (midState, arc1, arc2) = self.ComputeConnection(fromState, toState)

        # Save the information.
        self.fromState = fromState
        self.midState = midState
        self.toState = toState
        self.arc1 = arc1
        self.arc2 = arc2

    def ComputeConnection(self, fromState, toState):
        # Grab the starting and final coordinates.
        (x1, x2) = (fromState.x, toState.x)
        (y1, y2) = (fromState.y, toState.y)
        (t1, t2) = (fromState.t, toState.t)
        (s1, s2) = (fromState.s, toState.s)
        (c1, c2) = (fromState.c, toState.c)

        wb = turningradius

        a = 2 * s1 * s2 + 2 * c1 * c2 - 2
        b = 2 * ((x2 - x1) * (s2 + s1) - (y2 - y1) * (c2 + c1))
        c = (x2 - x1) ** 2 + (y2 - y1) ** 2

        if a == 0 and b == 0:
            r = np.Inf
            invR = 0
        elif a == 0 and b != 0:
            r = -c / b
            invR = 1 / r
        else:
            r1 = (-b - np.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
            r2 = (-b - np.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
            r = max(r1, r2)
            invR = 1 / r


        # Check for straight path (infinite radius).
        if invR == 0:
            # Straight line!
            tm = t1  # Theta at mid point
            xm = 0.5 * (x1 + x2)  # X coordinate at mid point
            ym = 0.5 * (y1 + y2)  # Y coordinate at mid point
            d1 = 0.5 * np.sqrt(c)  # Distance on first arc
            d2 = d1  # Distance on second arc

        else:
            # Use two arcs
            xa = x1 - 1 / invR * s1
            ya = y1 + 1 / invR * c1
            xb = x2 + 1 / invR * s2
            yb = y2 - 1 / invR * c2

            xm = 0.5 * (xa + xb)
            ym = 0.5 * (ya + yb)

            tm = np.arctan2((yb - ya), (xb - xa)) + (r / np.abs(r)) * np.pi / 2

            d1 = 1 / invR * (tm - t1)
            d2 = 1 / invR * (tm - t2)
        # Return the mid state and two arcs
        tansteer = invR * wb
        midState = State(xm, ym, tm)
        arc1 = Arc(fromState, midState, d1, tansteer)
        arc2 = Arc(midState, toState, d2, -tansteer)
        return (midState, arc1, arc2)

class Arc:
    def __init__(self, fromState, toState, distance, tansteer):
        # Remember the parameters.
        self.fromState = fromState
        self.toState = toState
        self.distance = distance  # can be nagative when backing up!
        self.tansteer = tansteer  # pos = turn left, neg = turn right

    def __repr__(self):
        return ("<Arc %s to %s, distance %5.2f m, steer %5.2f deg>" %
                (self.fromState, self.toState, self.distance,
                 np.rad2deg(np.arctan(self.tansteer))))

    # Return the absolute length.
    def Length(self):
        return abs(self.distance)

    # Return an intermediate state, d along arc
    def IntermediateState(self, d):
        if self.tansteer == 0:
            return State(self.fromState.x + self.fromState.c * d,
                         self.fromState.y + self.fromState.s * d,
                         self.fromState.t)
        else:
            r = turningradius / self.tansteer
            phi = d / r
            ds = np.sin(self.fromState.t + phi) - self.fromState.s
            dc = np.cos(self.fromState.t + phi) - self.fromState.c
            return State(self.fromState.x + r * ds,
                         self.fromState.y - r * dc,
                         self.fromState.t + phi)

    # Return a velocity between two states
    def Velocity(self,fromstate,tostate,time):
        return Velocity(fromstate.Distance(tostate)/time,AngleDiff(fromstate.t,tostate.t))

# Angular distance within +/- 180deg.
def AngleDiff(t1, t2):
    return (t1-t2) - 2.0*np.pi * round(0.5*(t1-t2)/np.pi)

class State:
    def __init__(self, x, y, theta):
        # Pre-compute the trigonometry.
        s = np.sin(theta)
        c = np.cos(theta)

        # Remember the state (x,y,theta).
        self.x = x
        self.y = y
        self.t = theta
        self.s = s
        self.c = c

    # Compute the relative distance to another state
    def Distance(self, other):
        return np.sqrt((self.x - other.x) ** 2 +
                       (self.y - other.y) ** 2)

class Velocity:
    def __init__(self, x, w):
        self.x = x
        self.w = w


class DriverObj:
    # CLOSE_ENOUGH_DISTANCE = 0.05 # m
    # CLOSE_ENOUGH_THETA = 0.5 # rad
    # OMEGA_P_CONST_SPIN = 1.0
    # OMEGA_P_CONST_DRIVE = 1.0 # 1/s
    # VEL_P_CONST = 0.5 # 1/s
    # MAX_VEL = 3.0 # m/s
    global turningradius
    turningradius = .010 # m
    start_sec = 0
    velocity = 1 # m/s

    currentplan = LocalPlan(State(0,0,0),State(0,0,0))

    def __init__(self):
        ## define class variables
        # velocity/angle change to publish
        self.vx = 0
        self.wz = 0
        # current
        self.cur_x = 0   # m
        self.cur_y = 0   # m
        self.cur_qz = 0   # rad
        self.cur_qw = 0
        self.cur_t = 0

        # desired
        self.des_x = 0   # m
        self.des_y = 0   # m
        self.des_qz = 0
        self.des_qw = 0
        self.des_t = 0

        ## initalize node
        # Create a publisher to send velocity commands.
        self.pub_velcmd = rospy.Publisher('/vel_cmd', Twist, queue_size=10)

        # Create a subscriber to listen to odometry commands commands.
        rospy.Subscriber('/odom', Odometry, self.cb_update_current)

        # Create a subscriber to listen to pose commands
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_update_desired)

    # Callback for /move_base_simple/goal, saves x, y, theta desired to class
    def cb_update_desired(self, msg):
        # Update desired x, y, theta, sin/cos
        self.des_x = msg.pose.position.x
        self.des_y = msg.pose.position.y
        self.des_qz = msg.pose.orientation.z
        self.des_qw = msg.pose.orientation.w

        s = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z)
        c = 1.0 - 2.0 * (msg.pose.pose.orientation.z ** 2)
        self.des_t = math.atan2(s,c)

        self.currentplan = LocalPlan(State(self.cur_x, self.cur_y,self.cur_t), State(self.des_x, self.des_y, self.des_t))
        self.start_sec = msg.header.stamp.to_sec()
        #self.start_nsec = msg.header.stamp.nsec






    # Callback for /odom, saves x, y, and theta current to class
    def cb_update_current(self, msg):
        # Update current x, y, theta, sin/cos
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_qz = msg.pose.pose.orientation.z
        self.cur_qw = msg.pose.pose.orientation.w

        s = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z)
        c = 1.0 - 2.0 * (msg.pose.pose.orientation.z ** 2)
        self.cur_t = math.atan2(s,c)
        self.cur_sec = msg.header.stamp.to_sec()

        # self.cur_nsec = msg.header.stamp.nsec

        dt = (self.cur_sec - self.start_sec)
        curd = dt*self.velocity
        if curd < self.currentplan.arc1.Length():
            curState = self.currentplan.arc1.IntermediateState(curd)
            nextState = self.currentplan.arc1.IntermediateState(curd + dt*self.velocity)
            newvelocity = self.currentplan.arc1.Velocity(curState,nextState,dt)

        else:
            prevd = self.currentplan.arc1.Length()
            curState = self.currentplan.arc2.IntermediateState(curd - prevd)
            nextState = self.currentplan.arc2.IntermediateState(curd - prevd + dt * self.velocity)
            newvelocity = self.currentplan.arc2.Velocity(curState, nextState, dt)

        # save message
        msg_velcmd = Twist()
        msg_velcmd.linear.x = newvelocity.x
        msg_velcmd.linear.y = 0.0
        msg_velcmd.linear.z = 0.0
        msg_velcmd.angular.x = 0.0
        msg_velcmd.angular.y = 0.0
        msg_velcmd.angular.z = newvelocity.w
        self.pub_velcmd.publish(msg_velcmd)


# Clamp a variable between a maximum and minimum (Saturated)
def clamp(x, min_x, max_x):
    clamped = max(min(max_x, x), min_x)
    return clamped


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
