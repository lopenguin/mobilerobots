#!/usr/bin/env python3
#
#   plan.py
#
#   Plan node.  This
#   (a) builds up a list of obstacles, allowing for moving obstacles/motions
#   (b) Fuses odometry and obstacle map for localization
#
#   Node:       /plan
#   Publish:    /vel_cmd          geometry_msgs/Twist
#   Subscribe:  /odom                   Odometry
#               /move_base_simple/goal  geometry_msgs/PoseStamped
#               /scan                   sensor_msgs/LaserScan
#               /visualization_marker   visualization_msgs/Marker
#

import math
import rospy
import numpy as np
from State import State, SegmentCrossSegment

from geometry_msgs.msg  import Point, Quaternion, Twist
from geometry_msgs.msg  import PoseStamped
from sensor_msgs.msg    import LaserScan
from visualization_msgs.msg import Marker

class Plan():
    def __init__(self):
        self.vx = 0.0
        self.wz = 0.0
        self.walls = []
        self.wallNum = 10

        # current
        self.cur_state = State.Empty()
        # desired
        self.des_states = [State.Empty(), State(1.0, 1.0, 0.0)]

        ## Publishers
        self.pub_velcmd = rospy.Publisher('/vel_cmd', Twist, queue_size=10)

        ## Subscribers
        rospy.Subscriber('/pose', PoseStamped, self.cb_pose)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_desired)
        rospy.Subscriber('/scan', LaserScan, self.cb_scan)
        rospy.Subscriber('/viz_instantaneous_lines', Marker, self.cb_walls)

    def cb_walls(self, msg):
        if (self.wallNum == 10):
            self.wallNum = 0
        else:
            self.wallNum += 1
            return
        # for now, just add each inst. wall to the list
        ptList = msg.points
        self.walls = []
        for i in range(0, len(ptList), 2):
            line = ((ptList[i].x, ptList[i].y), (ptList[i+1].x, ptList[i+1].y))
            self.walls.append(line)
            

    def cb_pose(self, msg):
        # assert (msg.header.frame_id == 'map'), "Message not in map frame"
        # Update current x, y, theta, sin/cos
        self.cur_state = State.FromPose(msg.pose)
        if (len(self.des_states) == 0):
            return
        new_des = self.des_states[0]

        line = ((self.cur_state.x, self.cur_state.y), (new_des.x, new_des.y))
        bad = False
        for wall in self.walls:
            if (SegmentCrossSegment(line, wall)):
                bad = True
                break
        
        if not bad:
            self.gotoPoint2()
            msg_velcmd = Twist()
            msg_velcmd.linear.x = self.vx
            msg_velcmd.linear.y = 0.0
            msg_velcmd.linear.z = 0.0
            msg_velcmd.angular.x = 0.0
            msg_velcmd.angular.y = 0.0
            msg_velcmd.angular.z = self.wz
            self.pub_velcmd.publish(msg_velcmd)
        else:
            msg_velcmd = Twist()
            msg_velcmd.linear.x = 0.0
            msg_velcmd.linear.y = 0.0
            msg_velcmd.linear.z = 0.0
            msg_velcmd.angular.x = 0.0
            msg_velcmd.angular.y = 0.0
            msg_velcmd.angular.z = 0.0
            self.pub_velcmd.publish(msg_velcmd)

    def cb_desired(self, msg):
        # assert (msg.header.frame_id == 'map'), "Message not in map frame"
        new_des = State.FromPose(msg.pose)
        # Clear the waypoint queue
        self.des_states.clear()
        self.des_states.append(new_des)


    def cb_scan(self, msg):
        # ranges = msg.ranges
        # minTheta = msg.min_angle
        # maxTheta = msg.max_angle
        pass


    # constants for gotoPoint2
    DRIVE_TIME_CONSTANT = 5.0
    MAX_DRIVE_SPEED = 0.75
    TURNING_TIME_CONSTANT = 2.0
    MAX_TURN_SPEED = 0.5 # rad/s

    CLOSE_ENOUGH_DISTANCE = 0.05 # m
    CLOSE_ENOUGH_THETA = 0.5
    def gotoPoint2(self):
        nwaypoints = len(self.des_states)
        if (nwaypoints == 0):
            return
            
        if (nwaypoints > 1):
            # consider skipping to the next waypoint
            if (self.cur_state.dist(self.des_states[0]) < 0.5):
                self.des_states.pop(0)

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
            self.vx = clamp(d*math.cos(theta_diff)/self.DRIVE_TIME_CONSTANT, 0 , self.MAX_DRIVE_SPEED)
            self.wz = clamp(theta_diff / self.TURNING_TIME_CONSTANT, -self.MAX_TURN_SPEED, self.MAX_TURN_SPEED)



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
    rospy.init_node('plan')

    # Instantiate the Odometry object
    plan = Plan()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Planning has begun...")
    rospy.spin()
    rospy.loginfo("Plan stopped.")