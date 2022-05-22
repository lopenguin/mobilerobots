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
from State import State

from geometry_msgs.msg  import Point, Quaternion, Twist
from geometry_msgs.msg  import PoseStamped
from sensor_msgs.msg    import LaserScan


#
#   Simple Driver
#
class DriverObj():
    def __init__(self):
        ## define class variables
        # velocity/angle change to publish
        self.vx = 0
        self.wz = 0
        # current
        self.cur_state = State.Empty()
        # desired
        self.des_states = [State.Empty(), State(1.0, 1.0, 0.0)]

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


    # constants for gotoPoint2
    DRIVE_TIME_CONSTANT = 5.0
    MAX_DRIVE_SPEED = 0.5
    TURNING_TIME_CONSTANT = 2.0
    MAX_TURN_SPEED = 0.5 # rad/s

    CLOSE_ENOUGH_DISTANCE = 0.05 # m
    CLOSE_ENOUGH_THETA = 0.5

    def gotoPoint2(self):
        if (len(self.des_states) == 0):
            return

        des_state = self.des_states[0]
        des_x = des_state.x
        des_y = des_state.y
        des_t = des_state.t

        cur_x = self.cur_state.x
        cur_y = self.cur_state.y
        cur_t = self.cur_state.t



        d = des_state.dist(self.cur_state)

        if d < self.CLOSE_ENOUGH_DISTANCE:
            self.vx = 0
            theta_diff_des = angleDiff(des_t, cur_t)
            self.wz = clamp(theta_diff_des / self.TURNING_TIME_CONSTANT, -self.MAX_TURN_SPEED, self.MAX_TURN_SPEED)

            if theta_diff_des < self.CLOSE_ENOUGH_THETA:
                self.wz = 0
                self.des_states.pop(0)
                print("Pop!")

        else:
            phi = math.atan2(des_y - cur_y, des_x - cur_x)
            theta_diff = angleDiff(phi, cur_t)
            self.vx = clamp(d*math.cos(theta_diff)/self.DRIVE_TIME_CONSTANT, 0.0 , self.MAX_DRIVE_SPEED)
            self.wz = clamp(theta_diff / self.TURNING_TIME_CONSTANT, -self.MAX_TURN_SPEED, self.MAX_TURN_SPEED)


    '''CALLBACK FUNCTIONS'''
    # Callback for /move_base_simple/goal
    def cb_update_desired(self, msg):
        assert (msg.header.frame_id == 'map'), "Message not in map frame"
        # Clear the waypoint queue
        self.des_states.clear()
        self.des_states.append(State.FromPose(msg.pose))

        self.stopped = False


    # Callback for /odom
    def cb_update_current(self, msg):
        assert (msg.header.frame_id == 'map'), "Message not in map frame"
        # Update current x, y, theta, sin/cos
        self.cur_state = State.FromPose(msg.pose)
        # print(self.cur_state.x)
        # print(self.cur_state.y)
        # print(self.cur_state.t)

        if (not self.stopped):
            self.gotoPoint2()
        msg_velcmd = Twist()
        msg_velcmd.linear.x = self.vx
        msg_velcmd.linear.y = 0.0
        msg_velcmd.linear.z = 0.0
        msg_velcmd.angular.x = 0.0
        msg_velcmd.angular.y = 0.0
        msg_velcmd.angular.z = self.wz
        self.pub_velcmd.publish(msg_velcmd)

    ## Callback for /scan
    # constants
    ESTOP_OFFSET_MULT = 0.85 # 1/s(DANGER ZONE)
    ESTOP_OFFSET_MIN = 0.05 # m
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
                # self.wz = 0
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
    rospy.init_node('astar_driver')

    # Instantiate the Driver object
    simpledriver = DriverObj()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Driver spinning...")
    rospy.spin()
    rospy.loginfo("Driver stopped.")
